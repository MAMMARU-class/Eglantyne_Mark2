#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "Eglantyne";
const char* pass = "Eglantyne";

IPAddress serverIP(192, 168, 3, 11);
const uint16_t serverPort = 80;
WiFiClient client;

const unsigned long TIMEOUT = 2000;
bool connected = false;

void connect();

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, pass);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
}

unsigned long lastPing = 0;
void loop() {
    // ensure connection
    if(!client.connected()){
        connect();
        lastPing = millis();
    }

    // send msg
    if (client.available()){
        lastPing = millis();
        String msg = client.readStringUntil('\n');

        if (msg == "ping\r"){
            client.println("0,0,255");
        }
    }

    // handle timeout
    if (millis() - lastPing > TIMEOUT){
        Serial.println("Connection timed out, reconnecting...");
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
        client.stop();
    }
}

void connect(){
    if (client.connect(serverIP, serverPort)) {
        Serial.println("Connected to server");
        neopixelWrite(RGB_BUILTIN, 0, 255, 0);
    } else {
        Serial.println("Connection failed");
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
    }
}
