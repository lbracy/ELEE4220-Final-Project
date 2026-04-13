#include <Arduino.h>

// ---------------- PIN DEFINITIONS (TEENSY) ----------------
#define V_HALL     A0
#define CURR_PIN A1
#define PWM_PIN    10   // good PWM pin on Teensy

// ---------------- Controller Gains ----------------
#define Kp_inner 10.0f
#define Ki_inner 10.0f

// ---------------- ACS712 Parameters ----------------
#define ACS_SENSITIVITY  0.066f
#define V_DIVIDER_RATIO  1.0f

// ---------------- SS49E Hall Sensor ----------------
#define HALL_SENSITIVITY_V_PER_GAUSS 0.0014f

// ---------------- Filters ----------------
#define CURRENT_ALPHA 0.05f
#define HALL_ALPHA    0.30f

// ---------------- Timing ----------------
#define CURRENT_SAMPLE_US  100    // 10 kHz
#define HALL_SAMPLE_US     1000   // 1 kHz
#define PRINT_US           100000 // 10 Hz

// ---------------- Timing Objects ----------------
elapsedMicros currentTimer;
elapsedMicros hallTimer;
elapsedMicros printTimer;

// ---------------- State ----------------
int   duty              = 0;
float i_ref             = 0.7f;
float i_meas            = 0.0f;
float B_gauss           = 0.0f;
float hall_voltage_filt = 0.0f;

float ACS_OFFSET  = 0.0f;
float HALL_OFFSET = 0.0f;

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

// ---------------------------------------------------
int pi_controller(float i, float i_ref_val) {
  static float integral_i = 0.0f;
  const float dt = 0.0001f;  // 100 µs loop

  float error = i_ref_val - i;
  integral_i += error * dt;

  float u = Kp_inner * error + Ki_inner * integral_i;

  if (u > 255.0f) u = 255.0f;
  if (u < 0.0f)   u = 0.0f;

  return (int)u;
}

// ---------------------------------------------------
float calibrateZeroCurrentVoltage() {
  const int numSamples = 500;
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
  const int numSamples = 500;
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

  while (!Serial) {
    ; // wait for USB serial
  }

  pinMode(PWM_PIN, OUTPUT);
  pinMode(CURR_PIN, INPUT);
  pinMode(V_HALL, INPUT);

  // -------- ADC (Teensy advantage) --------
  analogReadResolution(12);
  analogReadAveraging(4);   // BIG noise reduction

  // -------- PWM --------
  analogWriteResolution(8);
  analogWriteFrequency(PWM_PIN, 10000);
  analogWrite(PWM_PIN, 0);

  delay(500);

  // -------- Calibration --------
  ACS_OFFSET  = calibrateZeroCurrentVoltage();
  HALL_OFFSET = calibrateHallZeroVoltage();
  hall_voltage_filt = HALL_OFFSET;

  Serial.println("READY");
}

// ---------------------------------------------------
void loop() {

  // -------- 10 kHz CURRENT LOOP --------
  if (currentTimer >= CURRENT_SAMPLE_US) {
    currentTimer = 0;

    float v_sensor = readACSVoltage();
    float i_inst   = (v_sensor - ACS_OFFSET) / ACS_SENSITIVITY;

    i_meas = CURRENT_ALPHA * i_inst +
             (1.0f - CURRENT_ALPHA) * i_meas;

    duty = pi_controller(i_meas, i_ref);

    static int last_duty = -1;
    if (duty != last_duty) {
      analogWrite(PWM_PIN, duty);
      last_duty = duty;
    }
  }

  // -------- 1 kHz HALL LOOP --------
  if (hallTimer >= HALL_SAMPLE_US) {
    hallTimer = 0;

    float v_hall = readHallVoltage();

    hall_voltage_filt =
        HALL_ALPHA * v_hall +
        (1.0f - HALL_ALPHA) * hall_voltage_filt;

    float delta = hall_voltage_filt - HALL_OFFSET;
    B_gauss = delta / HALL_SENSITIVITY_V_PER_GAUSS;
  }

  // -------- PRINT --------
  if (printTimer >= PRINT_US) {
    printTimer = 0;

    float pct = duty / 255.0f * 100.0f;

    Serial.print(pct, 1);   Serial.print(",");
    Serial.print(i_ref, 3); Serial.print(",");
    Serial.print(i_meas, 3);Serial.print(",");
    Serial.println(B_gauss, 1);
  }
}