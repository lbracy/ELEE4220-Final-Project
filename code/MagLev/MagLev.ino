#include <Arduino.h>

#define CURR_PIN  1
#define HALL_PIN  2
#define PWM_PIN   5

#define PWM_FREQ_HZ     10000

#define Kp_inner  22.0f
#define Ki_inner  52000.0f

static float Kp_outer = 0.0f;
static float Ki_outer = 0.0f;
static float Kd_outer = 0.0f;

static float feed_forward = 0.4f;

#define MAX_CURRENT         1.5f
#define MIN_CURRENT         0.00f

#define SHUNT_RESISTANCE    0.1f
#define OPAMP_GAIN          40.0f
#define ACS_SENSITIVITY     (SHUNT_RESISTANCE * OPAMP_GAIN)

#define CURRENT_SAMPLE_US   100
#define OUTER_SAMPLE_US     1000
#define PRINT_US            80000
#define ARM_DELAY_MS        1000

static uint32_t currentTimer = 0;
static uint32_t outerTimer   = 0;
static uint32_t printTimer   = 0;

static bool systemArmed = false;

static int   duty   = 0;
static float i_ref  = 0.0f;
static float i_meas = 0.0f;

static float pos_ref_target = 2.15f;
static float prev_pos_error = 0.0f;
static float outer_integral = 0.0f;

static float inner_p_term     = 0.0f;
static float inner_i_term     = 0.0f;
static float inner_integral_i = 0.0f;

static float outer_pos_meas   = 0.0f;
static float outer_error      = 0.0f;
static float outer_p_term     = 0.0f;
static float outer_i_term     = 0.0f;
static float outer_d_term     = 0.0f;

static float readACSVoltage() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(CURR_PIN);
    }
    float avgRaw = sum / 16.0f;
    return (avgRaw / 4095.0f) * 3.3f;
}

static float readHallVoltage() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(HALL_PIN);
    }
    float avgRaw = sum / 16.0f;
    return (avgRaw / 4095.0f) * 3.3f;
}

static void writePWM(int d) {
    analogWrite(PWM_PIN, constrain(d, 0, 255));
}

static void pid_controller_outer() {
    const float dt = OUTER_SAMPLE_US / 1000000.0f;

    float raw_pos = readHallVoltage();

    static float filtered_pos = -1.0f;
    const float ALPHA = 0.15f;
    if (filtered_pos < 0.0f) {
        filtered_pos = raw_pos;
    }
    filtered_pos = (ALPHA * raw_pos) + ((1.0f - ALPHA) * filtered_pos);
    outer_pos_meas = filtered_pos;

    static float deriv_filtered = -1.0f;
    const float ALPHA_D = 0.05f;
    if (deriv_filtered < 0.0f) {
        deriv_filtered = raw_pos;
    }
    deriv_filtered = (ALPHA_D * raw_pos) + ((1.0f - ALPHA_D) * deriv_filtered);

    outer_error = pos_ref_target - outer_pos_meas;

    static bool first_run = true;
    if (first_run) {
        prev_pos_error = outer_error;
        first_run = false;
    }

    outer_p_term = Kp_outer * outer_error;

    outer_integral += outer_error * dt;
    float max_i_windup = MAX_CURRENT / (Ki_outer > 0 ? Ki_outer : 1.0f);
    outer_integral = constrain(outer_integral, -max_i_windup, max_i_windup);
    outer_i_term = Ki_outer * outer_integral;

    float deriv_error = pos_ref_target - deriv_filtered;
    outer_d_term = Kd_outer * (deriv_error - prev_pos_error) / dt;
    prev_pos_error = deriv_error;

    float desired_i = outer_p_term + outer_i_term + outer_d_term + feed_forward;

    i_ref = constrain(desired_i, MIN_CURRENT, MAX_CURRENT);
}

static int pi_controller_inner(float i, float i_ref_val) {
    const float dt = CURRENT_SAMPLE_US / 1000000.0f;
    float error = i_ref_val - i;

    inner_p_term = Kp_inner * error;
    float potential_u = inner_p_term + (Ki_inner * inner_integral_i);

    if (!((potential_u >= 255.0f && error > 0.0f) ||
          (potential_u <=   0.0f && error < 0.0f))) {
        inner_integral_i += error * dt;
    }

    inner_i_term = Ki_inner * inner_integral_i;
    float u = inner_p_term + inner_i_term;

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
    Serial.println("Commands: P<val>=Kp, I<val>=Ki, D<val>=Kd, R<val>=target, F<val>=feedforward");
    Serial.println("Examples: P8.0  I5.0  D0.0  R2.19  F0.40");
}

void loop() {
    if (!systemArmed) {
        writePWM(0);
        return;
    }

    uint32_t now = micros();

    if ((uint32_t)(now - outerTimer) >= OUTER_SAMPLE_US) {
        outerTimer = now;
        pid_controller_outer();
    }

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

    now = micros();
    if ((uint32_t)(now - printTimer) >= PRINT_US) {
        printTimer = now;

        if (Serial.available()) {
            String s = Serial.readStringUntil('\n');
            s.trim();
            if (s.length() > 1) {
                char cmd = s.charAt(0);
                float val = s.substring(1).toFloat();

                if (cmd == 'p' || cmd == 'P') {
                    Kp_outer = val;
                    Serial.print("\n>> Kp_outer: ");
                    Serial.println(Kp_outer, 6);
                } else if (cmd == 'i' || cmd == 'I') {
                    Ki_outer = val;
                    outer_integral = 0.0f;
                    Serial.print("\n>> Ki_outer: ");
                    Serial.println(Ki_outer, 6);
                } else if (cmd == 'd' || cmd == 'D') {
                    Kd_outer = val;
                    Serial.print("\n>> Kd_outer: ");
                    Serial.println(Kd_outer, 6);
                } else if (cmd == 'r' || cmd == 'R') {
                    pos_ref_target = val;
                    Serial.print("\n>> Target: ");
                    Serial.println(pos_ref_target, 4);
                } else if (cmd == 'f' || cmd == 'F') {
                    feed_forward = val;
                    Serial.print("\n>> Feed-forward: ");
                    Serial.println(feed_forward, 4);
                } else {
                    Serial.println("\n>> Unknown command. Use P, I, D, R, or F.");
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
        Serial.print(" | FF:");    Serial.print(feed_forward,   3); Serial.print(",");
        Serial.print(" | PWM%:");  Serial.println(pct,          1);
    }
}
