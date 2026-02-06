#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <IcsBaseClass.h>
#include <IcsHardSerialClass.h>

// ==========================
// motor settings
// ==========================
#define ICS_BAUDRATE 1250000
#define ICS_TIMEOUT 10
#define myEN1 16
#define myTX1 17
#define myRX1 18
IcsHardSerialClass krs1(&Serial1, myEN1, ICS_BAUDRATE, ICS_TIMEOUT, myRX1, myTX1);

// ==========================
// timing
// ==========================
const unsigned long PING_INTERVAL = 1000;
const unsigned long TIMEOUT       = 5000;

// ==========================
// ESP-NOW
// ==========================
uint8_t clientMac[6];
bool clientRegistered = false;

// ==========================
// state
// ==========================
int angle = 0;
unsigned long lastPing  = 0;
unsigned long lastReply = 0;

// ==========================
// ESP-NOW callbacks
// ==========================
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
    memcpy(clientMac, mac_addr, 6);
    lastReply = millis();

    if (!clientRegistered) {
        esp_now_peer_info_t peer{};
        memcpy(peer.peer_addr, clientMac, 6);
        peer.channel = 0;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
        clientRegistered = true;

        Serial.println("ESP-NOW client registered");
        neopixelWrite(RGB_BUILTIN, 0, 255, 0);
    }

    char msg[32];
    int n = min(len, (int)sizeof(msg) - 1);
    memcpy(msg, data, n);
    msg[n] = '\0';

    Serial.print("Received: ");
    Serial.println(msg);

    angle = atoi(msg);
    krs1.setPos(1, angle);
}

void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // optional: transmission status
}

// ==========================
// setup
// ==========================
void setup() {
    Serial.begin(115200);

    neopixelWrite(RGB_BUILTIN, 255, 0, 0);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (1);
    }

    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSend);

    Serial.println("ESP-NOW server ready");

    krs1.begin();

    Serial.println(WiFi.macAddress());
}

// ==========================
// loop
// ==========================
void loop() {
    // timeout handling (connection-like behavior)
    if (clientRegistered && millis() - lastReply > TIMEOUT) {
        Serial.println("timeout");
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
        esp_now_del_peer(clientMac);
        clientRegistered = false;
    }

    // send ping
    if (clientRegistered && millis() - lastPing > PING_INTERVAL) {
        const char *ping = "ping";
        esp_now_send(clientMac, (uint8_t *)ping, strlen(ping) + 1);
        lastPing = millis();
    }
}
