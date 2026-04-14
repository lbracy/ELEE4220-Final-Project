#include <Arduino.h>

// ---------------- PIN DEFINITIONS (TEENSY) ----------------
#define V_HALL     A0
#define CURR_PIN   A1
#define PWM_PIN    10   // Connected to L298N IN1 (IN2 is grounded)

// ---------------- OUTER LOOP GAINS (Position / Maglev) ----
/*
#define Kp_outer 0.015f
#define Ki_outer 0.00001f
#define Kd_outer 0.0009f 
#define B_TARGET -272.0f
*/
#define Kp_outer 0.004f      // Low P to prevent "launching"
#define Ki_outer 0.00001f
#define Kd_outer 0.001f      // High D to provide "friction"
#define B_TARGET -270.0f     // Further away for more reaction time

// ---------------- INNER LOOP GAINS (Current) --------------
#define Kp_inner 0.57f
#define Ki_inner 250.0f //723.53f

// ---------------- System Limits ---------------------------
#define MAX_CURRENT 1.0f
#define MIN_CURRENT 0.0f
#define FEED_FORWARD_CURRENT 0.35f

// ---------------- Sensor Parameters -----------------------
#define ACS_SENSITIVITY  0.066f
#define V_DIVIDER_RATIO  1.0f
#define HALL_SENSITIVITY_V_PER_GAUSS 0.0014f

// ---------------- Filters ---------------------------------
#define CURRENT_ALPHA 0.05f
#define HALL_ALPHA    0.85f

// ---------------- Timing ----------------------------------
#define CURRENT_SAMPLE_US  100    // 10 kHz inner loop
#define HALL_SAMPLE_US     300 // 1000   // 1 kHz outer loop
#define PRINT_US           20000   // 50 Hz telemetry
#define ARM_DELAY_MS       1000   // wait after user turns on supply

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

// ---------------- Arm State -------------------------------
bool systemArmed = false;

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
  // // If the outer loop wants zero current, wipe the memory of the inner loop
  // if (i_ref_val <= 0.001f) {
  //   integral_i = 0;
  //   return 0;
  // }
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
  const float dt = 0.0003f; //0.001f; // 1 ms loop

  position_error = b_meas - b_target;

  integral_outer += position_error * dt;

  if (integral_outer > 1000.0f) integral_outer = 1000.0f; 
  if (integral_outer < -1000.0f) integral_outer = -1000.0f;

  float d_error = (position_error - last_error) / dt;
  last_error = position_error;

  outer_p_term = Kp_outer * position_error;
  outer_i_term = Ki_outer * integral_outer;
  outer_d_term = Kd_outer * d_error;

  float new_i_ref = FEED_FORWARD_CURRENT + outer_p_term + outer_i_term + outer_d_term;

  if (new_i_ref > MAX_CURRENT) new_i_ref = MAX_CURRENT;
  if (new_i_ref < MIN_CURRENT) new_i_ref = MIN_CURRENT;

  return new_i_ref;
}

// ---------------- RESET CONTROL STATES --------------------
void resetControllerState() {
  i_ref = 0.0f;
  duty = 0;
  i_meas = 0.0f;

  outer_p_term = 0.0f;
  outer_i_term = 0.0f;
  outer_d_term = 0.0f;
  position_error = 0.0f;

  analogWrite(PWM_PIN, 0);

  currentTimer = 0;
  hallTimer = 0;
  printTimer = 0;
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

// ---------------- WAIT FOR USER TO ARM --------------------
void waitForUserArm() {
  Serial.println();
  Serial.println("TURN ON POWER SUPPLY.");
  Serial.println("When power supply is ON, press 's' then Enter to start control.");
  Serial.println("Controller is DISARMED. PWM = 0 while waiting.");

  while (true) {
    analogWrite(PWM_PIN, 0);

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

  resetControllerState();
  waitForUserArm();

  systemArmed = true;
  resetControllerState();

  Serial.println("CONTROL ENABLED.");
}

// ---------------------------------------------------
void loop() {

  // If not armed, do nothing
  if (!systemArmed) {
    analogWrite(PWM_PIN, 0);
    return;
  }

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

    i_ref = pid_controller_outer(B_gauss, B_TARGET);
  }

  // -------- PRINT --------
  if (printTimer >= PRINT_US) {
    printTimer = 0;
    
    float pct = duty / 255.0f * 100.0f;
    /*
    /* plotting for serial_logger.py */ 
    Serial.print(B_TARGET, 1);   Serial.print(",");
    Serial.print(B_gauss, 1);    Serial.print(",");
    Serial.print(position_error, 1); Serial.print(",");
    Serial.print(outer_p_term, 3);   Serial.print(",");
    Serial.print(outer_i_term, 3);   Serial.print(",");
    Serial.print(outer_d_term, 3);   Serial.print(",");
    Serial.print(i_ref, 3);      Serial.print(",");
    Serial.print(i_meas, 3);     Serial.print(",");
    Serial.println(pct, 1);
    */
    Serial.print("B_Tgt:"); Serial.print(B_TARGET, 1);  Serial.print(",");
    Serial.print(" | B_Ms:");  Serial.print(B_gauss, 1);   Serial.print(",");
    Serial.print(" | Err:");   Serial.print(position_error, 1); Serial.print(",");
    Serial.print(" | P:");     Serial.print(outer_p_term, 3); Serial.print(",");
    Serial.print(" | I:");     Serial.print(outer_i_term, 3); Serial.print(",");
    Serial.print(" | D:");     Serial.print(outer_d_term, 3); Serial.print(",");
    Serial.print(" | iRef:");  Serial.print(i_ref, 3);  Serial.print(",");
    Serial.print(" | iMs:");   Serial.print(i_meas, 3); Serial.print(",");
    Serial.print(" | PWM%:");  Serial.println(pct, 1);
   
  }
}