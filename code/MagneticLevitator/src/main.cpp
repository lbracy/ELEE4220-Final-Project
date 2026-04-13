#include <Arduino.h>

// ---------------- PIN DEFINITIONS (TEENSY) ----------------
#define V_HALL     A0
#define CURR_PIN   A1
#define PWM_PIN    10   // Connected to L298N IN1 (IN2 is grounded)

// ---------------- OUTER LOOP GAINS (Position / Maglev) ----
#define Kp_outer 0.1f    // Spring
#define Ki_outer 0.0000f // Gap closer (KEEP THIS TINY)
#define Kd_outer 0.0009f // Shock absorber
#define B_TARGET -375.0f // Your ideal hover point

// ---------------- INNER LOOP GAINS (Current) --------------
#define Kp_inner 0.57f
#define Ki_inner 723.53f

// ---------------- System Limits ---------------------------
#define MAX_CURRENT 2.0f  // Keep below L298N max
#define MIN_CURRENT 0.0f  // Hard floor because IN2 is grounded
#define FEED_FORWARD_CURRENT 0.2f // Baseline current to fight gravity

// ---------------- Sensor Parameters -----------------------
#define ACS_SENSITIVITY  0.066f
#define V_DIVIDER_RATIO  1.0f
#define HALL_SENSITIVITY_V_PER_GAUSS 0.0014f

// ---------------- Filters ---------------------------------
#define CURRENT_ALPHA 0.05f
#define HALL_ALPHA    0.30f

// ---------------- Timing ----------------------------------
#define CURRENT_SAMPLE_US  100    // 10 kHz inner loop
#define HALL_SAMPLE_US     1000   // 1 kHz outer loop
#define PRINT_US           100000 // 10 Hz telemetry

// ---------------- Timing Objects --------------------------
elapsedMicros currentTimer;
elapsedMicros hallTimer;
elapsedMicros printTimer;

// ---------------- State -----------------------------------
int   duty              = 0;
float i_ref             = 0.0f;
float i_meas            = 0.0f;
float B_gauss           = 0.0f;
float hall_voltage_filt = 0.0f;

float ACS_OFFSET  = 0.0f;
float HALL_OFFSET = 0.0f;

// ---------------- Telemetry State -------------------------
float outer_p_term = 0.0f;
float outer_i_term = 0.0f;
float outer_d_term = 0.0f;
float position_error = 0.0f;

// ---------------------------------------------------
float readACSVoltage() {
  int raw = analogRead(CURR_PIN);
  float v = (raw / 4095.0f) * 3.3f;
  return v / V_DIVIDER_RATIO;
}

float readHallVoltage() {
  int raw = analogRead(V_HALL);
  return (raw / 4095.0f) * 3.3f;
}

// ---------------- INNER LOOP (PI) --------------------------
int pi_controller_inner(float i, float i_ref_val) {
  static float integral_i = 0.0f;
  const float dt = 0.0001f;  // 100 µs loop

  float error = i_ref_val - i;
  integral_i += error * dt;

  float u = Kp_inner * error + Ki_inner * integral_i;

  // --- Anti-Windup Clamp (0 to 255 only) ---
  if (u > 255.0f) {
    u = 255.0f;
    integral_i -= error * dt; 
  }
  else if (u < 0.0f) {
    u = 0.0f;
    integral_i -= error * dt; 
  }

  return (int)u;
}

// ---------------- OUTER LOOP (PID) --------------------------
float pid_controller_outer(float b_meas, float b_target) {
  static float last_error = 0.0f;
  static float integral_outer = 0.0f;
  const float dt = 0.001f; // 1 ms loop

  // 1. Proportional
  position_error = b_meas - b_target;
  
  // 2. Integral
  integral_outer += position_error * dt;

  // AGGRESSIVE ANTI-WINDUP FOR MAGLEV
  if (integral_outer > 1000.0f) integral_outer = 1000.0f; 
  if (integral_outer < -1000.0f) integral_outer = -1000.0f;

  // 3. Derivative
  float d_error = (position_error - last_error) / dt;
  last_error = position_error;

  // Save to telemetry globals
  outer_p_term = Kp_outer * position_error;
  outer_i_term = Ki_outer * integral_outer;
  outer_d_term = Kd_outer * d_error;

  // PID Math + Feed Forward
  float new_i_ref = FEED_FORWARD_CURRENT + outer_p_term + outer_i_term + outer_d_term;

  // Clamp the requested current for the L298N hardware
  if (new_i_ref > MAX_CURRENT) new_i_ref = MAX_CURRENT;
  if (new_i_ref < MIN_CURRENT) new_i_ref = MIN_CURRENT;

  return new_i_ref;
}

// ---------------------------------------------------
float calibrateZeroCurrentVoltage() {
  const int numSamples = 250;
  long sum = 0;
  Serial.println("=== ACS712 Calibration ===");
  delay(1000);
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(CURR_PIN);
    delay(2);
  }
  float avg = sum / (float)numSamples;
  float v   = (avg / 4095.0f) * 3.3f;
  Serial.print("ACS offset = ");
  Serial.println(v, 4);
  return v;
}

float calibrateHallZeroVoltage() {
  const int numSamples = 250;
  long sum = 0;
  Serial.println("=== Hall Calibration ===");
  delay(1000);
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(V_HALL);
    delay(2);
  }
  float avg = sum / (float)numSamples;
  float v   = (avg / 4095.0f) * 3.3f;
  Serial.print("Hall offset = ");
  Serial.println(v, 4);
  return v;
}

// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PWM_PIN, OUTPUT);
  pinMode(CURR_PIN, INPUT);
  pinMode(V_HALL, INPUT);

  analogReadResolution(12);
  analogReadAveraging(4);   

  analogWriteResolution(8);
  analogWriteFrequency(PWM_PIN, 10000);
  analogWrite(PWM_PIN, 0);

  delay(500);

  ACS_OFFSET  = calibrateZeroCurrentVoltage();
  HALL_OFFSET = calibrateHallZeroVoltage();
  hall_voltage_filt = HALL_OFFSET;

  Serial.println("READY. Uni-Directional Outer Loop Engaged.");
}

// ---------------------------------------------------
void loop() {

  // -------- 10 kHz CURRENT LOOP (Inner) --------
  if (currentTimer >= CURRENT_SAMPLE_US) {
    currentTimer = 0;

    float v_sensor = readACSVoltage();
    // Negative sign maintained to fix sensor polarity
    float i_inst   = -(v_sensor - ACS_OFFSET) / ACS_SENSITIVITY;

    i_meas = (CURRENT_ALPHA * i_inst + (1.0f - CURRENT_ALPHA) * i_meas);

    duty = pi_controller_inner(i_meas, i_ref);

    static int last_duty = -1;
    if (duty != last_duty) {
      analogWrite(PWM_PIN, duty);
      last_duty = duty;
    }
  }

  // -------- 1 kHz HALL LOOP (Outer) --------
  if (hallTimer >= HALL_SAMPLE_US) {
    hallTimer = 0;

    float v_hall = readHallVoltage();

    hall_voltage_filt = HALL_ALPHA * v_hall + (1.0f - HALL_ALPHA) * hall_voltage_filt;

    float delta = hall_voltage_filt - HALL_OFFSET;
    B_gauss = delta / HALL_SENSITIVITY_V_PER_GAUSS;

    // Outer Loop dynamically calculates the current we need right now!
    i_ref = pid_controller_outer(B_gauss, B_TARGET);
  }

  // -------- PRINT --------
  if (printTimer >= PRINT_US) {
    printTimer = 0;
    
    float pct = duty / 255.0f * 100.0f;

    // Outer Loop Health
    Serial.print("B_Tgt:"); Serial.print(B_TARGET, 1);  Serial.print(",");
    Serial.print(" | B_Ms:");  Serial.print(B_gauss, 1);   Serial.print(",");
    Serial.print(" | Err:");   Serial.print(position_error, 1); Serial.print(",");
    
    // PID Contributions
    Serial.print(" | P:"); Serial.print(outer_p_term, 3); Serial.print(",");
    Serial.print(" | I:"); Serial.print(outer_i_term, 3); Serial.print(",");
    Serial.print(" | D:"); Serial.print(outer_d_term, 3); Serial.print(",");

    // Inner Loop Health
    Serial.print(" | iRef:"); Serial.print(i_ref, 3);  Serial.print(",");
    Serial.print(" | iMs:");  Serial.print(i_meas, 3); Serial.print(",");
    Serial.print(" | PWM%:"); Serial.println(pct, 1);
  }
}