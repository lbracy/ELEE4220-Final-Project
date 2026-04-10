#include <Arduino.h>

#define I_COIL 35
#define V_HALL 34
#define PWM_PIN 25
// ESP32 PWM settings
#define PWM_FREQ 10000
#define PWM_RESOLUTION 8   // 8-bit: duty from 0 to 255

float hall_voltage = 0.0;
float coil_current = 0.0;

float readHallSensor();

void setup() {
  Serial.begin(115200);

  // Set up PWM on ESP32
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(10000);   // 10 kHz
}

void loop() {
  //hall_voltage = readHallSensor();
  //Serial.print("Hall Sensor Voltage: ");  
  //Serial.println(hall_voltage);
  delay(100); 

  // pwm stuff
  // for (int percent = 10; percent <= 100; percent += 10) {
  //   int duty = (percent * 255) / 100;   // convert percent to 8-bit duty

  //   analogWrite(PWM_PIN, 255);
  //   Serial.print("PWM Percent: ");
  //   Serial.print(percent);
  //   Serial.print("% | Duty: ");
  //   Serial.println(duty);

  //   delay(1000);
  // }
  Serial.println("Ramp complete. Turning PWM off.");
  analogWrite(PWM_PIN, 255);
  delay(3000);
}

float readHallSensor() {
  return analogRead(V_HALL);
}