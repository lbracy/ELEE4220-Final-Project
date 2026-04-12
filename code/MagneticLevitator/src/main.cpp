#include <Arduino.h>

#define V_HALL 34
#define PWM_PIN 25
#define SENSOR_PIN 26

// --- Controller Gains ---
#define Kp_inner 10.0f  // dummy vars
#define Ki_inner 10.0f  // dummy vars

// --- ACS712 Parameters ---
// sensitivity is exactly 66 mV/A
#define ACS_SENSITIVITY 0.066f 
// Calibrated zero-current voltage based on hardware test
#define ACS_OFFSET 2.303f        

#define V_DIVIDER_RATIO 1.0f 

int duty = 0; 
float i_ref = 0.0f;
float i_meas = 0.0f; // Global variable to hold the smoothed EMA average

float readHallSensor();

int inner_feedback_loop(float i, float i_ref) {
  static float integral_i = 0.0f;
  const float dt = 0.025f;   // 25 ms loop time (matches delay(25) below)

  float error = i_ref - i;

  // integrate error
  integral_i += error * dt;

  // PI control
  float u = Kp_inner * error + Ki_inner * integral_i;

  // clamp to PWM limits (0 to 255 for standard 8-bit analogWrite)
  if (u > 255.0f) {
    u = 255.0f;
  }
  if (u < 0.0f) {
    u = 0.0f;
  }

  return (int)u;
}

void setup() {
  Serial.begin(115200);

  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(10000);   // 10 kHz

  pinMode(SENSOR_PIN, INPUT);
  pinMode(V_HALL, INPUT);
  analogReadResolution(12);      // 12-bit ADC (0-4095)
}

void loop() {

  i_ref = 0.4f;

  // 1. Read the raw ADC value (0-4095)
  int sensorRaw = analogRead(SENSOR_PIN);
  
  // 2. Convert raw value to voltage at the ESP32 pin (0 - 3.3V)
  float v_pin = (sensorRaw / 4095.0f) * 3.3f;
  
  // 3. Reverse the voltage divider to find the actual voltage outputting from the ACS712
  float v_sensor = v_pin / V_DIVIDER_RATIO;
  
  // 4. Calculate instantaneous current using the 30A calibrated offset
  float instantaneous_i = (v_sensor - ACS_OFFSET) / ACS_SENSITIVITY;

  // 5. APPLY EMA FILTER (Smooths out the 10 kHz PWM noise)
  i_meas = (0.1f * instantaneous_i) + (0.9f * i_meas);

  // 6. Run the inner loop using the SMOOTHED current
  duty = inner_feedback_loop(i_meas, i_ref);
  analogWrite(PWM_PIN, duty);

  // 7. Serial Output for debugging/plotting
  Serial.print("Duty: ");
  Serial.print(duty);
  Serial.print(" | Raw ADC: ");
  Serial.print(sensorRaw);
  Serial.print(" | ACS Voltage: ");
  Serial.print(v_sensor, 3);
  Serial.print(" V");
  Serial.print(" | Inst Current: ");
  Serial.print(instantaneous_i, 3);
  Serial.print(" A");
  Serial.print(" | Smoothed Current: ");
  Serial.print(i_meas, 3);
  Serial.print(" A");
  Serial.print(" | Hall Voltage: ");
  Serial.print((readHallSensor() * (3.3f / 4095.0f)), 3);
  Serial.println(" V");

  delay(25);
}

float readHallSensor() {
  return analogRead(V_HALL);
}