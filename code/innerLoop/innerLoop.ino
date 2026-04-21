/*
 * MagneticLevitator_Teensy41.cpp
 * Ported to Teensy 4.1
 *
 * Key hardware changes from ESP32 version:
 * - analogReadResolution set to 12-bit (Teensy 4.1 default is 10-bit)
 * - analogReadAveraging available on Teensy — used for hardware noise reduction
 * - LEDC peripheral removed — Teensy uses analogWrite() for PWM natively
 * - PWM frequency set via analogWriteFrequency()
 * - ADC is 3.3V referenced, 12-bit: raw/4095 * 3.3V same as ESP32
 */

#include <Arduino.h>

// ======================================================================
// PIN DEFINITIONS  (Teensy 4.1)
// ======================================================================
#define CURR_PIN  A4   // Any analog pin — Teensy 4.1 all analog pins are ADC-capable
#define PWM_PIN   2    // Any PWM-capable digital pin

// ======================================================================
// PWM CONFIGURATION
// ======================================================================
#define PWM_FREQ_HZ     10000

// ======================================================================
// INNER LOOP GAINS  (Current PI)
// ======================================================================
#define Kp_inner 100.0f
#define Ki_inner 750.0f

// ======================================================================
// SYSTEM LIMITS
// ======================================================================
#define MAX_CURRENT         1.5f
#define MIN_CURRENT         0.00f

// ======================================================================
// SENSOR PARAMETERS
// ======================================================================
//
// Current sense chain:  I → R3 (0.1Ω shunt) → U2 LM324N (gain=40) → ADC
//   ACS_SENSITIVITY = R_shunt × Gain = 0.1 × 40 = 4.0 V/A
#define SHUNT_RESISTANCE    0.1f    // R3 in Ohms
#define OPAMP_GAIN          40.0f  // 1 + (39kΩ / 1kΩ)
#define ACS_SENSITIVITY     (SHUNT_RESISTANCE * OPAMP_GAIN)  // = 4.0 V/A
#define V_DIVIDER_RATIO     1.0f

// ======================================================================
// FILTERS
// ======================================================================
#define CURRENT_ALPHA 0.85f

// ======================================================================
// TIMING  (microseconds)
// ======================================================================
#define CURRENT_SAMPLE_US   100     // 10 kHz inner loop
#define PRINT_US            80000   // ~12 Hz telemetry + serial poll
#define ARM_DELAY_MS        1000    // Pause after user arms system

// ======================================================================
// TIMING
// ======================================================================
static uint32_t currentTimer = 0;
static uint32_t printTimer   = 0;

// ======================================================================
// STATE
// ======================================================================
static int   duty   = 0;
static float i_ref  = 0.0f;
static float i_meas = 0.0f;

static float ACS_OFFSET = 0.0f;

// Telemetry
static float inner_p_term     = 0.0f;
static float inner_i_term     = 0.0f;
static float inner_integral_i = 0.0f;

// Arm state
static bool systemArmed = false;

// ======================================================================
// HELPERS
// ======================================================================

// Read current-sense ADC channel → voltage (after op-amp)
static float readACSVoltage() {
    int   raw = analogRead(CURR_PIN);
    float v   = (raw / 4095.0f) * 3.3f;
    return v / 4;//V_DIVIDER_RATIO;
}

// Write PWM duty (0–255)
static void writePWM(int d) {
    analogWrite(PWM_PIN, constrain(d, 0, 255));
}

// ======================================================================
// INNER LOOP  — Current PI
// ======================================================================
static int pi_controller_inner(float i, float i_ref_val) {
    const float dt   = 0.0001f;    // 100 µs inner loop period
    float error      = i_ref_val - i;

    inner_p_term      = Kp_inner * error;
    float potential_u = inner_p_term + (Ki_inner * inner_integral_i);

    // Anti-windup: freeze integrator when output is saturated and error
    // would push it further into saturation
    if (!((potential_u >= 255.0f && error > 0.0f) ||
          (potential_u <=   0.0f && error < 0.0f))) {
        inner_integral_i += error * dt;
    }

    inner_i_term = Ki_inner * inner_integral_i;
    float u      = inner_p_term + inner_i_term;

    return (int)constrain(u, 0.0f, 255.0f);
}

// ======================================================================
// RESET CONTROLLER STATE
// ======================================================================
static void resetControllerState() {
    i_ref  = 0.0f;
    duty   = 0;
    i_meas = 0.0f;

    inner_p_term     = 0.0f;
    inner_i_term     = 0.0f;
    inner_integral_i = 0.0f;

    writePWM(0);

    currentTimer = micros();
    printTimer   = micros();
}

// ======================================================================
// CALIBRATION  — measure zero-current op-amp offset at startup
// ======================================================================
static float calibrateZeroCurrentVoltage() {
    const int N = 250;
    long sum = 0;
    Serial.println("=== Current Sense Calibration ===");
    delay(1000);
    for (int i = 0; i < N; i++) {
        sum += analogRead(CURR_PIN);
        delay(2);
    }
    float avg = sum / (float)N;
    float v   = (avg / 4095.0f) * 3.3f;
    Serial.print("Current offset voltage = ");
    Serial.println(v, 4);
    return v;
}

// ======================================================================
// ARM SEQUENCE  — wait for user to confirm power supply is on
// ======================================================================
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

    // Teensy 4.1 — set ADC to 12-bit to match ESP32 scaling (raw/4095)
    analogReadResolution(12);
    // Hardware averaging — 4 samples per read, reduces noise without software filter overhead
    //analogReadAveraging(4);

    // Set PWM frequency on the output pin
    analogWriteFrequency(PWM_PIN, PWM_FREQ_HZ);
    // analogWrite resolution defaults to 8-bit (0–255) on Teensy — matches our duty range
    analogWriteResolution(8);
    writePWM(0);

    delay(500);

    ACS_OFFSET = 0;//calibrateZeroCurrentVoltage();

    resetControllerState();
    waitForUserArm();

    systemArmed = true;

    Serial.println("CONTROL ENABLED.");
    Serial.println("Send a current setpoint in Amps (e.g. 0.4) + Enter to command i_ref.");
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

    // -------- 10 kHz CURRENT LOOP (Inner) --------------------------------
    if ((uint32_t)(now - currentTimer) >= CURRENT_SAMPLE_US) {
        currentTimer = now;

        float v_sensor = readACSVoltage();

        // Invert sign to correct sensor/wiring polarity (same as original)
        // float i_inst = -(v_sensor - ACS_OFFSET) / ACS_SENSITIVITY;

        // i_meas = CURRENT_ALPHA * i_inst + (1.0f - CURRENT_ALPHA) * i_meas;
        i_meas = v_sensor; // / 0.1f;

        duty = pi_controller_inner(i_meas, i_ref);

        static int last_duty = -1;
        if (duty != last_duty) {
            writePWM(duty);
            last_duty = duty;
        }
    }

    // -------- ~12 Hz TELEMETRY + SERIAL POLL -----------------------------
    // Serial is ONLY read here — never inside the 10 kHz loop
    now = micros();
    if ((uint32_t)(now - printTimer) >= PRINT_US) {
        printTimer = now;

        // --- Serial input: new i_ref ---
        if (Serial.available()) {
            String s = Serial.readStringUntil('\n');
            s.trim();
            if (s.length() > 0) {
                float requested = s.toFloat();
                i_ref = constrain(requested, MIN_CURRENT, MAX_CURRENT);
                inner_integral_i = 0.0f;   // reset integrator on new step
                Serial.print(">> i_ref set to ");
                Serial.print(i_ref, 3);
                Serial.println(" A");
            }
        }

        float pct = duty / 255.0f * 100.0f;

        // Human-readable serial output (uncomment CSV block below for serial_logger.py)
        Serial.print(" | iRef:");  Serial.print(i_ref,        3); Serial.print(",");
        Serial.print(" | iMs:");   Serial.print(i_meas,       3); Serial.print(",");
        Serial.print(" | iP:");    Serial.print(inner_p_term, 3); Serial.print(",");
        Serial.print(" | iI:");    Serial.print(inner_i_term, 3); Serial.print(",");
        Serial.print(" | PWM%:");  Serial.println(pct,        1);

        // CSV block for serial_logger.py — uncomment and comment out block above
        // Serial.print(i_ref,        3); Serial.print(",");
        // Serial.print(i_meas,       3); Serial.print(",");
        // Serial.print(inner_p_term, 3); Serial.print(",");
        // Serial.print(inner_i_term, 3); Serial.print(",");
        // Serial.println(pct,        1);
    }
}