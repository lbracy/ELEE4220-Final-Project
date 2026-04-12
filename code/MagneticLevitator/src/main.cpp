#include <Arduino.h>

#define I_COIL 35
#define V_HALL 34
#define PWM_PIN 25
#define SHUNT_PIN 26

#define OPAMP_GAIN 0.77f // gain is 7.7 based on measuring resistors 
#define Kp_inner 10.0f  // dummy vars
#define Ki_inner 10.0f  // dummy vars

float hall_voltage = 0.0;
int shunt_raw = 0;

// inner loop constants
int duty = 0; 
float i_ref = 0.0f;
float i_meas = 0.0f;
float e_i = 0.0f;

float readHallSensor();

int inner_feedback_loop(float i, float i_ref) {
  static float integral_i = 0.0f;
  const float dt = 0.25f;   // 250 ms loop time

  float error = i_ref - i;

  // integrate error
  integral_i += error * dt;

  // PI control
  float u = Kp_inner * error + Ki_inner * integral_i;

  // clamp to PWM limits
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

  pinMode(SHUNT_PIN, INPUT);
  pinMode(V_HALL, INPUT);
  analogReadResolution(12);
}

void loop() {

  i_ref = 0.4f;

  // int shuntRaw = analogReadMilliVolts(SHUNT_PIN);
  // float v_out = shuntRaw / 1000.0f;
  // float i_meas = v_out / OPAMP_GAIN;

  int shuntRaw = analogRead(SHUNT_PIN);
  float v_out = (shuntRaw / 4095.0f) * 3.3f;
  i_meas = v_out / OPAMP_GAIN;

  duty = inner_feedback_loop(i_meas, i_ref);
  analogWrite(PWM_PIN, duty);

  Serial.print("Duty: ");
  Serial.print(duty);
  Serial.print(" | Raw ADC Shunt: ");
  Serial.print(shuntRaw);
  Serial.print(" | Coil Current: ");
  Serial.print(i_meas, 3);
  Serial.print(" A");
  Serial.print(" | Shunt Voltage: ");
  Serial.print(v_out, 3);
  Serial.println(" V");

  delay(25);
}

float readHallSensor() {
  return analogRead(V_HALL);
}