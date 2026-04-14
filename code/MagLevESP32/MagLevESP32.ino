/*
 * MagneticLevitator_ESP32.cpp
 * Ported from Teensy to ESP32 WROOM-32D
 *
 * Key hardware changes from original Teensy version:
 *  - L298N replaced by N-MOSFET (Q1) on PCB — PWM drives gate directly via R4 (1k)
 *  - ACS712 current sensor replaced by 0.1Ω shunt (R3) + LM324N op-amp (U2)
 *  - LM324N feedback resistor R6 changed from 6.8k → 39k
 *
 * ---- ACS_SENSITIVITY recalculation ----
 *  The LM324N is wired as a non-inverting amplifier:
 *    Rin  = R4 = 1kΩ  (input resistor)
 *    Rf   = R6 = 39kΩ (feedback resistor — CHANGED)
 *    Gain = 1 + Rf/Rin = 1 + 39/1 = 40
 *
 *  Effective sensitivity = R_shunt × Gain = 0.1Ω × 40 = 4.0 V/A
 *
 *  NOTE: At MAX_CURRENT = 1.0A the op-amp output reaches 4.0V, which EXCEEDS
 *  the ESP32 ADC's 3.3V input limit. Either:
 *    (a) Reduce MAX_CURRENT to 0.80A  ← safe limit with 3.3V rail
 *    (b) Add a voltage divider after U2 output (e.g. 10k / 22k → ×0.69 → 2.76V/A)
 *        and update ACS_SENSITIVITY accordingly
 *  The code below uses option (a) with MAX_CURRENT = 0.80A as a safe default.
 *  Adjust once you have confirmed your hardware headroom.
 *
 * ---- ESP32 ADC note ----
 *  Always use ADC1 pins (GPIO32–39). ADC2 cannot be used when WiFi is active.
 *  GPIO34 and GPIO35 are input-only — ideal for sensor inputs.
 *  The ESP32 ADC has known non-linearity near 0V and 3.3V; keep sensor
 *  mid-range for best accuracy.
 *
 * ---- PWM note ----
 *  Uses the ESP32 LEDC peripheral via the Arduino-ESP32 3.x API (ledcAttach /
 *  ledcWrite). If you are on arduino-esp32 2.x, see the #ifdef block below.
 */

#include <Arduino.h>

// ======================================================================
// PIN DEFINITIONS  (ESP32 WROOM-32D)
// ======================================================================
#define V_HALL    36    // ADC1_CH6  — Hall sensor breakout output
#define CURR_PIN  39    // ADC1_CH7  — LM324N (U2) current-sense output
// PWM output to MOSFET gate (via R4 1kΩ gate resistor on PCB)
#define PWM_PIN   25    // Any GPIO with output capability

// ======================================================================
// LEDC PWM CONFIGURATION
// ======================================================================
#define PWM_FREQ_HZ     10000
#define PWM_RESOLUTION  8       // 8-bit (0–255 duty range)

// ======================================================================
// OUTER LOOP GAINS  (Position / Maglev PID)
// ======================================================================
#define Kp_outer 0.17f
#define Ki_outer 0.01f
#define Kd_outer 0.001f
#define B_TARGET -125.0f    // Target field in Gauss

// ======================================================================
// INNER LOOP GAINS  (Current PI)
// ======================================================================
#define Kp_inner 0.57f
#define Ki_inner 2133.3f

// ======================================================================
// SYSTEM LIMITS
// ======================================================================

#define MAX_CURRENT         0.80f
#define MIN_CURRENT         0.00f
#define FEED_FORWARD_CURRENT 0.55f

// ======================================================================
// SENSOR PARAMETERS
// ======================================================================
//
// Current sense chain:  I → R3 (0.1Ω shunt) → U2 LM324N (gain=40) → ADC
//   ACS_SENSITIVITY = R_shunt × Gain = 0.1 × 40 = 4.0 V/A
#define SHUNT_RESISTANCE    0.1f    // R3 in Ohms
#define OPAMP_GAIN          40.0f  // 1 + (39kΩ / 1kΩ)
#define ACS_SENSITIVITY     (SHUNT_RESISTANCE * OPAMP_GAIN)  // = 4.0 V/A

// Hall sensor (SS49E or similar linear Hall IC on breakout board J6)
// The breakout outputs directly — no op-amp in signal path.
// 1.4 mV/Gauss is the typical SS49E sensitivity at 3.3V supply.
#define V_DIVIDER_RATIO              1.0f
#define HALL_SENSITIVITY_V_PER_GAUSS 0.0014f

// ======================================================================
// FILTERS
// ======================================================================
#define CURRENT_ALPHA 0.85f
#define HALL_ALPHA    0.80f

// ======================================================================
// TIMING  (microseconds)
// ======================================================================
#define CURRENT_SAMPLE_US   100     // 10 kHz inner loop
#define HALL_SAMPLE_US      200     //  5 kHz outer loop
#define PRINT_US            20000   // 50 Hz telemetry
#define ARM_DELAY_MS        1000    // Pause after user arms system

// ======================================================================
// TIMING  —  replaces Teensy's elapsedMicros type
// ======================================================================
static uint32_t currentTimer = 0;
static uint32_t hallTimer    = 0;
static uint32_t printTimer   = 0;

// ======================================================================
// STATE
// ======================================================================
static int   duty              = 0;
static float i_ref             = 0.0f;
static float i_meas            = 0.0f;
static float B_gauss           = 0.0f;
static float hall_voltage_filt = 0.0f;

static float ACS_OFFSET  = 0.0f;
static float HALL_OFFSET = 0.0f;

// Telemetry
static float outer_p_term   = 0.0f;
static float outer_i_term   = 0.0f;
static float outer_d_term   = 0.0f;
static float position_error = 0.0f;
static float inner_p_term   = 0.0f;
static float inner_i_term   = 0.0f;
static float inner_integral_i = 0.0f;

// Arm / safety state
static bool systemArmed = false;
static bool isOvershot  = false;

// ======================================================================
// HELPERS
// ======================================================================

// Read current-sense ADC channel → voltage (after op-amp)
static float readACSVoltage() {
    int   raw = analogRead(CURR_PIN);
    float v   = (raw / 4095.0f) * 3.3f;
    return v / V_DIVIDER_RATIO;
}

// Read Hall-sensor ADC channel → voltage
static float readHallVoltage() {
    int raw = analogRead(V_HALL);
    return (raw / 4095.0f) * 3.3f;
}

// Write PWM duty (0–255) via LEDC
static void writePWM(int d) {
    ledcWrite(PWM_PIN, (uint32_t)constrain(d, 0, 255));
}

// ======================================================================
// INNER LOOP  — Current PI
// ======================================================================
static int pi_controller_inner(float i, float i_ref_val) {
    // Hard reset when safety kill is active (i_ref = 0 while over-shot)
    if (isOvershot && i_ref_val <= 0.001f) {
        inner_integral_i = 0.0f;
        inner_p_term     = 0.0f;
        inner_i_term     = 0.0f;
        return 0;
    }

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
// OUTER LOOP  — Field / Position PID
// ======================================================================
static float pid_controller_outer(float b_meas, float b_target) {
    static float last_error     = 0.0f;
    static float integral_outer = 0.0f;

    const float dt = 0.0002f;   // 200 µs outer loop period

    position_error   = b_meas - b_target;
    integral_outer  += position_error * dt;

    // Integrator clamp
    integral_outer = constrain(integral_outer, -500.0f, 500.0f);

    float d_error = (position_error - last_error) / dt;
    last_error    = position_error;

    outer_p_term = Kp_outer * position_error;
    outer_i_term = Ki_outer * integral_outer;
    outer_d_term = Kd_outer * d_error;

    float new_i_ref = FEED_FORWARD_CURRENT + outer_p_term + outer_i_term + outer_d_term;
    return constrain(new_i_ref, MIN_CURRENT, MAX_CURRENT);
}

// ======================================================================
// RESET CONTROLLER STATE
// ======================================================================
static void resetControllerState() {
    i_ref    = 0.0f;
    duty     = 0;
    i_meas   = 0.0f;

    outer_p_term   = 0.0f;
    outer_i_term   = 0.0f;
    outer_d_term   = 0.0f;
    position_error = 0.0f;

    writePWM(0);

    currentTimer = micros();
    hallTimer    = micros();
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
// CALIBRATION  — measure Hall sensor quiescent voltage at startup
// ======================================================================
static float calibrateHallZeroVoltage() {
    const int N = 250;
    long sum = 0;
    Serial.println("=== Hall Sensor Calibration ===");
    delay(1000);
    for (int i = 0; i < N; i++) {
        sum += analogRead(V_HALL);
        delay(2);
    }
    float avg = sum / (float)N;
    float v   = (avg / 4095.0f) * 3.3f;
    Serial.print("Hall offset voltage = ");
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
    while (!Serial) { /* wait for USB serial on some boards */ }

    // Configure ADC
    // ESP32 ADC is 12-bit by default; analogReadResolution is still valid here.
    analogReadResolution(12);
    // NOTE: analogReadAveraging() is a Teensy-only function.
    //       For noise reduction on ESP32, we rely on the software EMA filters
    //       (CURRENT_ALPHA, HALL_ALPHA) already in the control loops.
    //       If you need hardware averaging, use the ESP32 IDF adc_oneshot API.

    // Configure PWM output via LEDC
    // arduino-esp32 3.x API — single-call attach:
    ledcAttach(PWM_PIN, PWM_FREQ_HZ, PWM_RESOLUTION);
    writePWM(0);

    // If you are on arduino-esp32 2.x, comment out the ledcAttach line above
    // and uncomment these three lines instead:
    //   ledcSetup(0, PWM_FREQ_HZ, PWM_RESOLUTION);
    //   ledcAttachPin(PWM_PIN, 0);
    //   // then replace ledcWrite(PWM_PIN, duty) with ledcWrite(0, duty)

    delay(500);

    ACS_OFFSET        = calibrateZeroCurrentVoltage();
    HALL_OFFSET       = calibrateHallZeroVoltage();
    hall_voltage_filt = HALL_OFFSET;

    resetControllerState();
    waitForUserArm();

    systemArmed = true;
    resetControllerState();

    Serial.println("CONTROL ENABLED.");
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
        float i_inst = -(v_sensor - ACS_OFFSET) / ACS_SENSITIVITY;

        i_meas = CURRENT_ALPHA * i_inst + (1.0f - CURRENT_ALPHA) * i_meas;

        duty = pi_controller_inner(i_meas, i_ref);

        static int last_duty = -1;
        if (duty != last_duty) {
            writePWM(duty);
            last_duty = duty;
        }
    }

    // -------- 5 kHz HALL LOOP (Outer) ------------------------------------
    now = micros();
    if ((uint32_t)(now - hallTimer) >= HALL_SAMPLE_US) {
        hallTimer = now;

        float v_hall = readHallVoltage();
        hall_voltage_filt = HALL_ALPHA * v_hall + (1.0f - HALL_ALPHA) * hall_voltage_filt;

        float delta = hall_voltage_filt - HALL_OFFSET;
        B_gauss     = delta / HALL_SENSITIVITY_V_PER_GAUSS;

        i_ref = pid_controller_outer(B_gauss, B_TARGET);

        // Safety kill: if ball is closer to coil than target, cut current
        // (more negative B_gauss = closer to coil)
        if (B_gauss < B_TARGET - 80.0f) {
            isOvershot = true;
            i_ref = 0.0f;
        } else {
            isOvershot = false;
        }
    }

    // -------- 50 Hz TELEMETRY --------------------------------------------
    now = micros();
    if ((uint32_t)(now - printTimer) >= PRINT_US) {
        printTimer = now;

        float pct = duty / 255.0f * 100.0f;

        // Human-readable serial output (uncomment CSV block below for serial_logger.py)
        Serial.print("B_Tgt:"); Serial.print(B_TARGET, 1);       Serial.print(",");
        Serial.print(" | B_Ms:");  Serial.print(B_gauss, 1);        Serial.print(",");
        Serial.print(" | Err:");   Serial.print(position_error, 1); Serial.print(",");
        Serial.print(" | P:");     Serial.print(outer_p_term, 3);   Serial.print(",");
        Serial.print(" | I:");     Serial.print(outer_i_term, 3);   Serial.print(",");
        Serial.print(" | D:");     Serial.print(outer_d_term, 3);   Serial.print(",");
        Serial.print(" | iP:");    Serial.print(inner_p_term, 3);   Serial.print(",");
        Serial.print(" | iI:");    Serial.print(inner_i_term, 3);   Serial.print(",");
        Serial.print(" | iRef:");  Serial.print(i_ref, 3);          Serial.print(",");
        Serial.print(" | iMs:");   Serial.print(i_meas, 3);         Serial.print(",");
        Serial.print(" | PWM%:");  Serial.println(pct, 1);

        // CSV block for serial_logger.py — uncomment and comment out block above
        // Serial.print(B_TARGET, 1);       Serial.print(",");
        // Serial.print(B_gauss, 1);        Serial.print(",");
        // Serial.print(position_error, 1); Serial.print(",");
        // Serial.print(outer_p_term, 3);   Serial.print(",");
        // Serial.print(outer_i_term, 3);   Serial.print(",");
        // Serial.print(outer_d_term, 3);   Serial.print(",");
        // Serial.print(inner_p_term, 3);   Serial.print(",");
        // Serial.print(inner_i_term, 3);   Serial.print(",");
        // Serial.print(i_ref, 3);          Serial.print(",");
        // Serial.print(i_meas, 3);         Serial.print(",");
        // Serial.println(pct, 1);
    }
}