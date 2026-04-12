#include <Arduino.h>

#define V_HALL     34
#define PWM_PIN    25
#define SENSOR_PIN 26
#define pin_test1 13
#define pin_test2 12

// ---------------- Controller Gains ----------------
#define Kp_inner 10.0f
#define Ki_inner 10.0f

// ---------------- ACS712 Parameters ----------------
#define ACS_SENSITIVITY  0.066f   // ACS712-30A: 66 mV/A
#define V_DIVIDER_RATIO  1.0f     // change if you use a voltage divider

// ---------------- SS49E Hall Sensor Parameters ----------------
// Datasheet: 1.4 mV/Gauss typical sensitivity at 25°C
// NOTE: SS49E outputs 2.5V at zero field (powered at 5V on HW-495).
// Positive field readings above ~800 Gauss will clip at 3.3V ESP32 ADC input.
#define HALL_SENSITIVITY_V_PER_GAUSS 0.0014f   // 1.4 mV/Gauss = 0.0014 V/Gauss

// ---------------- Filter Coefficients ----------------
// EMA: higher alpha = faster response, less smoothing
// at 10 kHz: τ = (1-alpha)/alpha * 0.1ms
// at  1 kHz: τ = (1-alpha)/alpha * 1.0ms
#define CURRENT_ALPHA 0.05f   // τ ≈ 0.95ms at 10 kHz — rejects PWM noise
#define HALL_ALPHA    0.30f   // τ ≈ 2.3ms  at  1 kHz — fast but smoothed

// ---------------- Timing ----------------
#define CURRENT_SAMPLE_US  100     // 10 kHz
#define HALL_SAMPLE_US     1000    //  1 kHz
#define PRINT_US           100000  // 10 Hz

// ---------------------------------------------------
int   duty             = 0;
float i_ref            = 0.48f;
float i_meas           = 0.0f;   // EMA-filtered current (A)
float hall_voltage_filt = 0.0f;  // EMA-filtered hall voltage (V)
float B_gauss          = 0.0f;   // magnetic field (Gauss)

float ACS_OFFSET  = 0.0f;        // calibrated zero-current voltage (V)
float HALL_OFFSET = 0.0f;        // calibrated zero-field hall voltage (V)

unsigned long last_current_sample = 0;
unsigned long last_hall_sample    = 0;
unsigned long last_print          = 0;

// ---------------- Function Declarations ----------------
float calibrateZeroCurrentVoltage();
float calibrateHallZeroVoltage();
float readACSVoltage();
float readHallVoltage();
int   inner_feedback_loop(float i, float i_ref);

// ---------------------------------------------------
int inner_feedback_loop(float i, float i_ref) {
  static float integral_i = 0.0f;
  const float dt = 0.0001f;   // 100 µs — matches 10 kHz sample rate

  float error = i_ref - i;
  integral_i += error * dt;

  float u = Kp_inner * error + Ki_inner * integral_i;

  if (u > 255.0f) u = 255.0f;
  if (u < 0.0f)   u = 0.0f;

  return (int)u;
}

// ---------------------------------------------------
float readACSVoltage() {
  int   raw     = analogRead(SENSOR_PIN);
  float v_pin   = (raw / 4095.0f) * 3.3f;
  float v_sensor = v_pin / V_DIVIDER_RATIO;
  return v_sensor;
}

float readHallVoltage() {
  int raw = analogRead(V_HALL);
  return (raw / 4095.0f) * 3.3f;
}

// ---------------------------------------------------
float calibrateZeroCurrentVoltage() {
  const int numSamples = 500;
  long adcSum = 0;

  Serial.println("=== ACS712 Calibration ===");
  Serial.println("Ensure NO current is flowing. Sampling in 1s...");
  delay(1000);

  for (int i = 0; i < numSamples; i++) {
    adcSum += analogRead(SENSOR_PIN);
    delay(2);
  }

  float adcAvg  = adcSum / (float)numSamples;
  float v_pin   = (adcAvg / 4095.0f) * 3.3f;
  float v_sensor = v_pin / V_DIVIDER_RATIO;

  Serial.print("  ADC avg:      "); Serial.println(adcAvg, 2);
  Serial.print("  Zero voltage: "); Serial.print(v_sensor, 4); Serial.println(" V");
  Serial.println("==========================");

  return v_sensor;
}

float calibrateHallZeroVoltage() {
  const int numSamples = 500;
  long adcSum = 0;

  Serial.println("=== SS49E Hall Calibration ===");
  Serial.println("Keep magnet away from sensor. Sampling in 1s...");
  delay(1000);

  for (int i = 0; i < numSamples; i++) {
    adcSum += analogRead(V_HALL);
    delay(2);
  }

  float adcAvg = adcSum / (float)numSamples;
  float v_hall = (adcAvg / 4095.0f) * 3.3f;

  Serial.print("  ADC avg:      "); Serial.println(adcAvg, 2);
  Serial.print("  Zero voltage: "); Serial.print(v_hall, 4); Serial.println(" V");
  Serial.println("  Note: SS49E null is ~2.5V at 5V supply.");
  Serial.println("  Positive field clipping occurs above ~3.3V (~800 Gauss).");
  Serial.println("==============================");

  return v_hall;
}

// ---------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PWM_PIN,    OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(V_HALL,     INPUT);
  pinMode(pin_test1, OUTPUT);
  pinMode(pin_test2, OUTPUT);

  analogReadResolution(12);
  analogWriteFrequency(10000);   // 10 kHz PWM

  analogWrite(PWM_PIN, 0);       // keep output off during calibration
  analogWrite(pin_test1, 0);
  analogWrite(pin_test2, 0);
  delay(500);

  ACS_OFFSET  = calibrateZeroCurrentVoltage();
  HALL_OFFSET = calibrateHallZeroVoltage();

  // seed filter to calibrated baselines
  i_meas            = 0.0f;
  hall_voltage_filt = HALL_OFFSET;

  Serial.println("\n=== Calibration Complete ===");
  Serial.print("  ACS_OFFSET  = "); Serial.print(ACS_OFFSET,  4); Serial.println(" V");
  Serial.print("  HALL_OFFSET = "); Serial.print(HALL_OFFSET, 4); Serial.println(" V");

  Serial.println("duty,i_ref_A,i_meas_A,B_gauss");
}

// ---------------------------------------------------
void loop() {
  unsigned long now = micros();

  // -------- 10 kHz: Current sampling & inner PI loop --------
  if (now - last_current_sample >= CURRENT_SAMPLE_US) {
    last_current_sample = now;

    float v_sensor       = readACSVoltage();
    float instantaneous_i = (v_sensor - ACS_OFFSET) / ACS_SENSITIVITY;
    i_meas = (CURRENT_ALPHA * instantaneous_i) + ((1.0f - CURRENT_ALPHA) * i_meas);

    duty = inner_feedback_loop(i_meas, i_ref);
    duty = 100;
    analogWrite(PWM_PIN, duty);
    analogWrite(pin_test1, duty);
    analogWrite(pin_test2, duty);
  }

  // -------- 1 kHz: Hall sampling & outer loop --------
  if (now - last_hall_sample >= HALL_SAMPLE_US) {
    last_hall_sample = now;

    float hall_voltage = readHallVoltage();
    hall_voltage_filt  = (HALL_ALPHA * hall_voltage) + ((1.0f - HALL_ALPHA) * hall_voltage_filt);

    float hall_delta = hall_voltage_filt - HALL_OFFSET;
    B_gauss = hall_delta / HALL_SENSITIVITY_V_PER_GAUSS;

    // TODO: plug outer loop here
    // i_ref = outerLoop(B_gauss);
  }

  // -------- 10 Hz: Serial print --------
  if (now - last_print >= PRINT_US) {
    last_print = now;
    float percentD = duty/255.0f * 100.0f;
    Serial.print(percentD);        Serial.print(",");
    Serial.print(i_ref, 3);    Serial.print(",");
    Serial.print(i_meas, 3);   Serial.print(",");
    Serial.println(B_gauss, 1);
    // Serial.print(readACSVoltage(), 4); Serial.print(",");  // raw V debug
    // Serial.println(ACS_OFFSET, 4);                         // offset debug
  }
}