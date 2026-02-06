#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ==========================
// timing
// ==========================
const unsigned long TIMEOUT = 5000;
unsigned long lastPing = 0;

// ==========================
// ESP-NOW
// ==========================
// Server の MAC アドレスを設定（必須
uint8_t serverMac[6] = { 0xDC, 0xB4, 0xD9, 0x06, 0xCA, 0x40 };

// ==========================
// volume settings（変更なし）
// ==========================
const int ANALOG_PIN = 1;

int analog_max = 4095;
int analog_min = 0;
int servo_max  = 11500;
int servo_min  = 3500;

int value = analog_min;
int a = 200;

// ==========================
// ESP-NOW callbacks
// ==========================
void onReceive(const uint8_t *mac_addr,
               const uint8_t *data,
               int len)
{
    char msg[16];
    int n = min(len, (int)sizeof(msg) - 1);
    memcpy(msg, data, n);
    msg[n] = '\0';

    if (strcmp(msg, "ping") == 0) {
        lastPing = millis();
        Serial.println("ping received");
        neopixelWrite(RGB_BUILTIN, 0, 0, 255);
    }
}

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // 送信結果は使わない（Wi-Fi版と同等）
}

// ==========================
// setup
// ==========================
void setup() {
    Serial.begin(115200);
    neopixelWrite(RGB_BUILTIN, 255, 0, 0);

    // ADC setting（変更なし）
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // ---------- ESP-NOW ----------
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (1);
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSend);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, serverMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("ESP-NOW client ready");
}

// ==========================
// loop
// ==========================
void loop() {
    // timeout handling（Wi-Fi版と同等）
    if (millis() - lastPing > TIMEOUT) {
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
    }

    int value = analogRead(ANALOG_PIN);

    int mapped = map(value,
                     analog_min - 200,
                     analog_max + 200,
                     servo_min,
                     servo_max);
    
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", mapped);

    esp_now_send(serverMac,
             (uint8_t *)buf,
             strlen(buf) + 1);

    delay(10);
}
