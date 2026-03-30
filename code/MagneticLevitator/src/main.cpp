#include <Arduino.h>

#define I_COIL 35
#define V_HALL 34
#define PWM_PIN 25

float hall_voltage = 0.0;
float coil_current = 0.0;

float readHallSensor();

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
}

void loop() {
 hall_voltage = readHallSensor();
  Serial.print("Hall Sensor Voltage: ");  
  Serial.println(hall_voltage);
  delay(100); 
}

float readHallSensor() {
  return analogRead(V_HALL);
}