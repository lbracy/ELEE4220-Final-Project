#include <Arduino.h>

// ======================================================================
// PIN DEFINITIONS  (Teensy 4.1)
// ======================================================================
#define CURR_PIN  1   // Current sense ADC
#define HALL_PIN  0   // Hall effect sensor ADC
#define PWM_PIN   5    // PWM output to electromagnet driver

// ======================================================================
// PWM CONFIGURATION
// ======================================================================
#define PWM_FREQ_HZ     10000

// ======================================================================
// INNER LOOP GAINS (Current PI)
// ======================================================================
#define Kp_inner 100.0f
#define Ki_inner 750.0f

// ======================================================================
// OUTER LOOP GAINS (Position PID)
// ======================================================================
static float Kp_outer = 0.5f;
static float Ki_outer = 0.01f;
static float Kd_outer = 0.0f;   // Start at 0 — tune P+I first, then add D carefully

// System mechanical equilibrium (Amps needed to balance gravity at hover)
// To calibrate: set P/I/D=0, manually command current via serial until magnet hovers,
// then set that value here.
#define I_BIAS 0.15f             

// ======================================================================
// SYSTEM LIMITS
// ======================================================================
#define MAX_CURRENT         1.5f
#define MIN_CURRENT         0.00f

// ======================================================================
// SENSOR PARAMETERS
// ======================================================================
#define SHUNT_RESISTANCE    0.1f    
#define OPAMP_GAIN          40.0f  
#define ACS_SENSITIVITY     (SHUNT_RESISTANCE * OPAMP_GAIN)  // = 4.0 V/A

// ======================================================================
// TIMING  (microseconds)
// ======================================================================
#define CURRENT_SAMPLE_US   100     // 10 kHz inner loop
#define OUTER_SAMPLE_US     1000    // 1 kHz outer loop
#define PRINT_US            80000   // 12.5 Hz telemetry
#define ARM_DELAY_MS        1000    // Pause after user arms system

// ======================================================================
// STATE & TIMERS
// ======================================================================
static uint32_t currentTimer = 0;
static uint32_t outerTimer   = 0;
static uint32_t printTimer   = 0;

static bool systemArmed = false;

// Shared State
static int   duty   = 0;
static float i_ref  = 0.0f;  // Commanded by Outer Loop, tracked by Inner Loop
static float i_meas = 0.0f;

// Outer Loop State
static float pos_ref_target = 2.0f; // Desired Hall sensor voltage (adjustable via Serial)
static float prev_pos_error = 0.0f;
static float outer_integral = 0.0f;

// Telemetry (Inner)
static float inner_p_term     = 0.0f;
static float inner_i_term     = 0.0f;
static float inner_integral_i = 0.0f;

// Telemetry (Outer)
static float outer_pos_meas   = 0.0f;
static float outer_error      = 0.0f;
static float outer_p_term     = 0.0f;
static float outer_i_term     = 0.0f;
static float outer_d_term     = 0.0f;


// ======================================================================
// HARDWARE HELPERS
// ======================================================================

static float readACSVoltage() {
    uint32_t sum = 0;
    for(int i = 0; i < 16; i++) {
        sum += analogRead(CURR_PIN);
    }
    float avgRaw = sum / 16.0f;
    return (avgRaw / 4095.0f) * 3.3f;
}

static float readHallVoltage() {
    uint32_t sum = 0;
    for(int i = 0; i < 16; i++) {
        sum += analogRead(HALL_PIN);
    }
    float avgRaw = sum / 16.0f;
    return (avgRaw / 4095.0f) * 3.3f;
}

static void writePWM(int d) {
    analogWrite(PWM_PIN, constrain(d, 0, 255));
}

// ======================================================================
// CONTROLLERS
// ======================================================================

// 1 kHz Outer Loop - Position PID
static void pid_controller_outer() {
    const float dt = OUTER_SAMPLE_US / 1000000.0f; // 0.001 sec
    
    // 1. Get the oversampled raw value
    float raw_pos = readHallVoltage();
    
    // 2a. EMA filter for P and I terms (moderate smoothing)
    static float filtered_pos = -1.0f;
    const float ALPHA = 0.15f;
    if (filtered_pos < 0.0f) {
        filtered_pos = raw_pos;
    }
    filtered_pos = (ALPHA * raw_pos) + ((1.0f - ALPHA) * filtered_pos);
    outer_pos_meas = filtered_pos;

    // 2b. Separate, heavier EMA filter dedicated to the derivative term.
    //     Differentiating a noisy signal amplifies noise proportional to frequency,
    //     so this filter is intentionally much stronger than the one used for P/I.
    //     Lower ALPHA_D = smoother D term but more phase lag. Start here and
    //     tune upward only if D response feels too sluggish.
    static float deriv_filtered = -1.0f;
    const float ALPHA_D = 0.05f;
    if (deriv_filtered < 0.0f) {
        deriv_filtered = raw_pos;
    }
    deriv_filtered = (ALPHA_D * raw_pos) + ((1.0f - ALPHA_D) * deriv_filtered);
    
    // 3. CORRECTED POLARITY FOR BOTTOM-UP SENSOR
    // If magnet drops -> voltage goes UP -> error becomes POSITIVE -> pulls harder
    outer_error = outer_pos_meas - pos_ref_target;

    // 4. FIX: Seed prev_pos_error on the very first call so the derivative term
    //    doesn't produce a huge spike from (error - 0) on the first tick.
    //    Must happen AFTER outer_error is computed.
    static bool first_run = true;
    if (first_run) {
        prev_pos_error = outer_error;
        first_run = false;
    }
    
    // Proportional
    outer_p_term = Kp_outer * outer_error;
    
    // Integral with anti-windup
    outer_integral += outer_error * dt;
    float max_i_windup = MAX_CURRENT / (Ki_outer > 0 ? Ki_outer : 1.0f);
    outer_integral = constrain(outer_integral, -max_i_windup, max_i_windup);
    outer_i_term = Ki_outer * outer_integral;
    
    // Derivative — computed from the heavily-filtered signal to suppress noise.
    // Note: prev_pos_error is now derived from deriv_filtered, not outer_error,
    // so the two quantities are consistent with each other.
    float deriv_error = deriv_filtered - pos_ref_target;
    outer_d_term = Kd_outer * (deriv_error - prev_pos_error) / dt;
    prev_pos_error = deriv_error;
    
    // Total PID output with feedforward bias
    float desired_i = outer_p_term + outer_i_term + outer_d_term + I_BIAS;
    
    // Command the inner loop
    i_ref = constrain(desired_i, MIN_CURRENT, MAX_CURRENT);
}

// 10 kHz Inner Loop — Current PI
static int pi_controller_inner(float i, float i_ref_val) {
    const float dt   = CURRENT_SAMPLE_US / 1000000.0f; // 0.0001 sec
    float error      = i_ref_val - i;

    inner_p_term      = Kp_inner * error;
    float potential_u = inner_p_term + (Ki_inner * inner_integral_i);

    // Anti-windup
    if (!((potential_u >= 255.0f && error > 0.0f) ||
          (potential_u <=   0.0f && error < 0.0f))) {
        inner_integral_i += error * dt;
    }

    inner_i_term = Ki_inner * inner_integral_i;
    float u      = inner_p_term + inner_i_term;

    return (int)constrain(u, 0.0f, 255.0f);
}

static void resetControllerState() {
    i_ref  = 0.0f;
    duty   = 0;
    i_meas = 0.0f;

    inner_p_term     = 0.0f;
    inner_i_term     = 0.0f;
    inner_integral_i = 0.0f;

    outer_integral   = 0.0f;
    prev_pos_error   = 0.0f;

    writePWM(0);

    uint32_t now = micros();
    currentTimer = now;
    outerTimer   = now;
    printTimer   = now;
}

static void waitForUserArm() {
    Serial.println();
    Serial.println("TURN ON POWER SUPPLY.");
    Serial.println("Press 's' then Enter once supply is ON to start control.");
    Serial.println("Controller is DISARMED. PWM = 0 while waiting.");

    while (true) {
        writePWM(0);
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 's' || c == 'S') {
                Serial.println("Start command received.");
                Serial.print("Waiting ");
                Serial.print(ARM_DELAY_MS);
                Serial.println(" ms before enabling control...");
                delay(ARM_DELAY_MS);
                break;
            }
        }
    }
}


// ======================================================================
// SETUP
// ======================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    analogReadResolution(12);
    analogWriteFrequency(PWM_PIN, PWM_FREQ_HZ);
    analogWriteResolution(PWM_PIN, 8);
    writePWM(0);

    delay(500);

    resetControllerState();
    waitForUserArm();

    systemArmed = true;

    Serial.println("CONTROL ENABLED.");
    Serial.println("Commands: P<val>=Kp, I<val>=Ki, D<val>=Kd, R<val>=target");
    Serial.println("Examples: P0.3  I0.00001  D0.005  R2.4");
}


// ======================================================================
// MAIN LOOP
// ======================================================================
void loop() {

    if (!systemArmed) {
        writePWM(0);
        return;
    }

    uint32_t now = micros();

    // -------- 1 kHz POSITION LOOP (Outer) --------------------------------
    if ((uint32_t)(now - outerTimer) >= OUTER_SAMPLE_US) {
        outerTimer = now;
        pid_controller_outer(); 
    }

    // -------- 10 kHz CURRENT LOOP (Inner) --------------------------------
    if ((uint32_t)(now - currentTimer) >= CURRENT_SAMPLE_US) {
        currentTimer = now;

        float v_sensor = readACSVoltage();
        i_meas = v_sensor / 4.0f;

        duty = pi_controller_inner(i_meas, i_ref);

        static int last_duty = -1;
        if (duty != last_duty) {
            writePWM(duty);
            last_duty = duty;
        }
    }

    // -------- 12.5 Hz TELEMETRY + SERIAL POLL ---------------------------
    now = micros();
    if ((uint32_t)(now - printTimer) >= PRINT_US) {
        printTimer = now;

        // --- Serial input: Command Parser ---
        if (Serial.available()) {
            String s = Serial.readStringUntil('\n');
            s.trim();
            if (s.length() > 1) {
                char cmd = s.charAt(0);
                float val = s.substring(1).toFloat();

                if (cmd == 'p' || cmd == 'P') {
                    Kp_outer = val;
                    Serial.print("\n>> Kp_outer updated to: "); 
                    Serial.println(Kp_outer, 6);
                } 
                else if (cmd == 'i' || cmd == 'I') {
                    Ki_outer = val;
                    outer_integral = 0.0f;  // Reset integrator to avoid windup jump on gain change
                    Serial.print("\n>> Ki_outer updated to: "); 
                    Serial.println(Ki_outer, 6);
                }
                else if (cmd == 'd' || cmd == 'D') {
                    Kd_outer = val;
                    Serial.print("\n>> Kd_outer updated to: "); 
                    Serial.println(Kd_outer, 6);
                } 
                else if (cmd == 'r' || cmd == 'R') {
                    pos_ref_target = val;
                    Serial.print("\n>> Hover Target (POS_REF) updated to: "); 
                    Serial.println(pos_ref_target, 4);
                } 
                else {
                    Serial.println("\n>> Unknown command. Use P, I, D, or R followed by a number.");
                    Serial.println("   Examples: P0.3  I0.00001  D0.005  R2.4");
                }
            }
        }

        float pct = duty / 255.0f * 100.0f;

        Serial.print(" | PosV:");  Serial.print(outer_pos_meas, 3); Serial.print(",");
        Serial.print(" | Err:");   Serial.print(outer_error,    3); Serial.print(",");
        Serial.print(" | oP:");    Serial.print(outer_p_term,   3); Serial.print(",");
        Serial.print(" | oD:");    Serial.print(outer_d_term,   3); Serial.print(",");
        Serial.print(" | iRef:");  Serial.print(i_ref,          3); Serial.print(",");
        Serial.print(" | iMs:");   Serial.print(i_meas,         3); Serial.print(",");
        Serial.print(" | PWM%:");  Serial.println(pct,          1);
    }

}
