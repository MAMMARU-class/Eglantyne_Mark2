#include <Arduino.h>

const int ANALOG_PIN = 1;  // GPIO1

void setup() {
    Serial.begin(115200);
    delay(500);

    // ADC設定（推奨）
    analogReadResolution(12);        // 0–4095
    analogSetAttenuation(ADC_11db);  // 0–3.3V
}

void loop() {
    int value = analogRead(ANALOG_PIN);
    Serial.println(value);
    delay(100);
}
