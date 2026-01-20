#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "Eglantyne";
const char* pass = "Eglantyne";

IPAddress serverIP(192, 168, 3, 11);
const uint16_t serverPort = 80;
WiFiClient client;

const unsigned long WIFI_RETRY_INTERVAL = 4000;
const unsigned long TIMEOUT = 2000;

unsigned long lastWiFiTry = 0;
unsigned long lastPing = 0;

void handle_WiFi();
void handle_conn();

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, pass);
    neopixelWrite(RGB_BUILTIN, 255, 0, 0);
}

void loop() {
    // ensure connection
    handle_WiFi();
    handle_conn();

    // receive ping
    if (client.available()){
        lastPing = millis();
        String msg = client.readStringUntil('\n');

        if (msg == "ping\r"){
            client.println("0,0,255");
        }
    }
}

void handle_WiFi(){
    // return if connected or not time yet
    if (WiFi.status() == WL_CONNECTED) return;
    if (millis() - lastWiFiTry < WIFI_RETRY_INTERVAL) return;

    neopixelWrite(RGB_BUILTIN, 255, 0, 0);
    Serial.println("Connecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, pass);

    lastWiFiTry = millis();
}

void handle_conn(){
    // return if WiFi disconnected or client connected or not time yet
    if (WiFi.status() != WL_CONNECTED) return;
    if (client.connected()) return;
    if (millis() - lastPing < TIMEOUT) return;

    // connect to server
    Serial.println("Connecting to server...");
    if (client.connect(serverIP, serverPort)) {
        Serial.println("Connected to server");
        neopixelWrite(RGB_BUILTIN, 0, 0, 255);
    } else {
        Serial.println("Connection failed");
        // change WiFi status
        WiFi.disconnect();
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
    }
}
