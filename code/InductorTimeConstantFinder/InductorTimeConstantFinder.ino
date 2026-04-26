#define CURR_PIN 1
#define PWM_PIN  5

void setup() {
    Serial.begin(115200);
    while(!Serial) {}
    
    analogReadResolution(12);
    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW); // Start OFF

    Serial.println("Send 't' to trigger step response...");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 't') {
            digitalWrite(PWM_PIN, HIGH); // Hit it with full voltage
            uint32_t start_time = micros();
            
            // Log for 50 milliseconds
            while (micros() - start_time < 50000) {
                uint32_t t = micros() - start_time;
                
                // Quick 4x oversampling
                int sum = analogRead(CURR_PIN) + analogRead(CURR_PIN) + 
                          analogRead(CURR_PIN) + analogRead(CURR_PIN);
                float v_sensor = ((sum / 4.0f) / 4095.0f) * 3.3f;
                float i_meas = v_sensor / 4.0f; // Assuming 4.0 V/A sensitivity
                
                Serial.print(t); // Time in microseconds
                Serial.print(",");
                Serial.println(i_meas, 3); // Current in Amps
            }
            digitalWrite(PWM_PIN, LOW); // Immediately turn off to prevent overheating
            Serial.println("TEST COMPLETE.");
        }
    }
}