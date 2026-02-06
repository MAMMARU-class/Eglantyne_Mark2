#include <Arduino.h>
#include <WiFi.h>
#include <IcsBaseClass.h>
#include <IcsHardSerialClass.h>

// multicore setting
TaskHandle_t thp[1];
void Core0a(void *args);

// connection settings
const char ssid[] = "Eglantyne";
const char pass[] = "Eglantyne";
const IPAddress ip(192, 168, 3, 11);
const IPAddress subnet(255, 255, 255, 0);
WiFiServer server(80);
WiFiClient client;

const unsigned long PING_INTERVAL = 1000;
const unsigned long TIMEOUT = 5000;

// motor settings
#define ICS_BAUDRATE 1250000
#define ICS_TIMEOUT 10

#define myEN1 16
#define myTX1 17
#define myRX1 18

void handle_conne();

void setup() {
    Serial.begin(115200);
    digitalWrite(LED_BUILTIN, LOW);

    WiFi.softAP(ssid, pass);
    delay(100);
    WiFi.softAPConfig(ip, ip, subnet);
    IPAddress myIP = WiFi.softAPIP();
    server.begin();
    Serial.print("SSID = ");
    Serial.println(ssid);
    Serial.print("IP address = ");
    Serial.println(myIP);
    Serial.println("server started");
    neopixelWrite(RGB_BUILTIN, 255, 0, 0);

    krs1.begin();
}

int angle = 0;

unsigned long lastPing = 0;
unsigned long lastReply = 0;
void loop() {
    handle_conne();

    // receive data
    if (client.available()) {
        String msg = client.readStringUntil('\n');
        lastReply = millis();
        Serial.print("Received: ");
        Serial.println(msg);

        angle = msg.toInt();
        krs1.setPos(1, angle);
    }

    // send ping
    if (millis() - lastPing > PING_INTERVAL){
        client.println("ping");
        lastPing = millis();
    }
}

void handle_conne(){
    if (millis() - lastReply > TIMEOUT){
        client.stop();
        Serial.println("timeout");
    }
    if (client && client.connected()) return;

    // Serial.println("disconnected");
    neopixelWrite(RGB_BUILTIN, 255, 0, 0);

    client = server.available();
    if(client){
        Serial.println("connected");
        neopixelWrite(RGB_BUILTIN, 0, 255, 0);
        lastReply = millis();
    }
}
