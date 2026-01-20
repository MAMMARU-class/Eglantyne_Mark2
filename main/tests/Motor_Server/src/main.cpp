#include <Arduino.h>
#include <WiFi.h>
#include <IcsBaseClass.h>
#include <IcsHardSerialClass.h>

// connection settings
const char ssid[] = "Eglantyne";
const char pass[] = "Eglantyne";
const IPAddress ip(192, 168, 3, 11);
const IPAddress subnet(255, 255, 255, 0);
WiFiServer server(80);
WiFiClient client;

const unsigned long PING_INTERVAL = 100;
const unsigned long TIMEOUT = 2000;

// motor settings
#define ICS_BAUDRATE 115200
#define ICS_TIMEOUT 10

#define myEN1 16
#define myTX1 17
#define myRX1 18

IcsHardSerialClass krs1(&Serial1, myEN1, ICS_BAUDRATE, ICS_TIMEOUT, myRX1, myTX1);

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

unsigned long lastPing = 0;
unsigned long lastReply = 0;
void loop() {
    // if client is not defined or connected, create new
    if(!client || !client.connected()){
        client = server.available();
        if(client){
            Serial.println("connected");
            neopixelWrite(RGB_BUILTIN, 0, 255, 0);
            lastReply = millis();
        }
    }

    // receive data
    if (client.available()) {
        String msg = client.readStringUntil('\n');
        lastReply = millis();
        Serial.print("Received: ");
        Serial.println(msg);

        krs1.setPos(1, msg.toInt());
    }

    // send ping
    if (millis() - lastPing > PING_INTERVAL){
        client.println("ping");
        lastPing = millis();
    }

    // handle timeout
    if (millis() - lastReply > TIMEOUT) {
        Serial.println("disconnected");
        neopixelWrite(RGB_BUILTIN, 255, 0, 0);
        client.stop();
    }
}
