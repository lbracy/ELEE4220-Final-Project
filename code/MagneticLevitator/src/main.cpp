#include <Arduino.h>

#define V_HALL     34
#define PWM_PIN    25
#define SENSOR_PIN 35

// ---------------- Controller Gains ----------------
#define Kp_inner 10.0f
#define Ki_inner 10.0f

// ---------------- ACS712 Parameters ----------------
#define ACS_SENSITIVITY  0.066f
#define V_DIVIDER_RATIO  1.0f

// ---------------- SS49E Hall Sensor Parameters ----------------
#define HALL_SENSITIVITY_V_PER_GAUSS 0.0014f

// ---------------- Filter Coefficients ----------------
#define CURRENT_ALPHA 0.05f
#define HALL_ALPHA    0.30f

// ---------------- Timing ----------------
#define CURRENT_SAMPLE_US  100    // 10 kHz
#define HALL_SAMPLE_US     1000   //  1 kHz
#define PRINT_US           100000 // 10 Hz

// ---------------------------------------------------
// ISR flags — set by timer, cleared by loop()
// ---------------------------------------------------
volatile bool do_current_sample = false;
volatile bool do_hall_sample    = false;

// ---------------------------------------------------
// Shared state
// ---------------------------------------------------
int   duty              = 0;
float i_ref             = 0.48f;
float i_meas            = 0.0f;
float B_gauss           = 0.0f;
float hall_voltage_filt = 0.0f;

float ACS_OFFSET  = 0.0f;
float HALL_OFFSET = 0.0f;

hw_timer_t* timer0 = NULL;
hw_timer_t* timer1 = NULL;

// ---------------------------------------------------
// ISRs — only set flags, nothing else
// ---------------------------------------------------
void IRAM_ATTR onCurrentTimer() {
  do_current_sample = true;
}

void IRAM_ATTR onHallTimer() {
  do_hall_sample = true;
}

// ---------------------------------------------------
float readACSVoltage() {
  int   raw    = analogRead(SENSOR_PIN);
  float v_pin  = (raw / 4095.0f) * 3.3f;
  return v_pin / V_DIVIDER_RATIO;
}

float readHallVoltage() {
  int raw = analogRead(V_HALL);
  return (raw / 4095.0f) * 3.3f;
}

int pi_controller(float i, float i_ref_val) {
  static float integral_i = 0.0f;
  const float dt = 0.0001f;

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
  long adcSum = 0;
  Serial.println("=== ACS712 Calibration ===");
  Serial.println("Ensure NO current is flowing. Sampling in 1s...");
  delay(1000);
  for (int i = 0; i < numSamples; i++) { adcSum += analogRead(SENSOR_PIN); delay(2); }
  float adcAvg   = adcSum / (float)numSamples;
  float v_sensor = (adcAvg / 4095.0f) * 3.3f / V_DIVIDER_RATIO;
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
  for (int i = 0; i < numSamples; i++) { adcSum += analogRead(V_HALL); delay(2); }
  float adcAvg = adcSum / (float)numSamples;
  float v_hall = (adcAvg / 4095.0f) * 3.3f;
  Serial.print("  ADC avg:      "); Serial.println(adcAvg, 2);
  Serial.print("  Zero voltage: "); Serial.print(v_hall, 4); Serial.println(" V");
  Serial.println("==============================");
  return v_hall;
}

// ---------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PWM_PIN,    OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(V_HALL,     INPUT);
  analogReadResolution(12);
  analogWriteFrequency(10000);
  analogWrite(PWM_PIN, 0);
  delay(500);

  ACS_OFFSET  = calibrateZeroCurrentVoltage();
  HALL_OFFSET = calibrateHallZeroVoltage();
  hall_voltage_filt = HALL_OFFSET;

  Serial.println("\n=== Calibration Complete ===");
  Serial.print("  ACS_OFFSET  = "); Serial.print(ACS_OFFSET,  4); Serial.println(" V");
  Serial.print("  HALL_OFFSET = "); Serial.print(HALL_OFFSET, 4); Serial.println(" V\n");
  Serial.println("duty_pct,i_ref_A,i_meas_A,B_gauss");

  // Start timers AFTER calibration
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onCurrentTimer, true);
  timerAlarmWrite(timer0, CURRENT_SAMPLE_US, true);
  timerAlarmEnable(timer0);

  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, &onHallTimer, true);
  timerAlarmWrite(timer1, HALL_SAMPLE_US, true);
  timerAlarmEnable(timer1);
}

// ---------------------------------------------------
// loop() does all the real work — ISRs just set flags
// ---------------------------------------------------
void loop() {

  // -------- 10 kHz: current sample + inner PI --------
  if (do_current_sample) {
    do_current_sample = false;

    float v_sensor        = readACSVoltage();
    float instantaneous_i = (v_sensor - ACS_OFFSET) / ACS_SENSITIVITY;
    i_meas = (CURRENT_ALPHA * instantaneous_i) + ((1.0f - CURRENT_ALPHA) * i_meas);

    //duty = pi_controller(i_meas, i_ref);
    duty = pi_controller(i_meas, i_ref);
    //duty = 100;
    static int last_duty = -1;
    if (duty != last_duty) {
      analogWrite(PWM_PIN, duty);
      last_duty = duty;
    }
  }

  // -------- 1 kHz: hall sample + outer loop --------
  if (do_hall_sample) {
    do_hall_sample = false;

    float hall_voltage = readHallVoltage();
    hall_voltage_filt  = (HALL_ALPHA * hall_voltage) + ((1.0f - HALL_ALPHA) * hall_voltage_filt);

    float hall_delta = hall_voltage_filt - HALL_OFFSET;
    B_gauss = hall_delta / HALL_SENSITIVITY_V_PER_GAUSS;

    // outer loop goes here:
    // i_ref = outerLoop(B_gauss);
  }

  // -------- 10 Hz: print --------
  static unsigned long last_print = 0;
  if (micros() - last_print >= PRINT_US) {
    last_print = micros();

    float pct = duty / 255.0f * 100.0f;
    Serial.print(pct, 1);   Serial.print(",");
    Serial.print(i_ref, 3); Serial.print(",");
    Serial.print(i_meas, 3);Serial.print(",");
    Serial.println(B_gauss, 1);
  }
}
