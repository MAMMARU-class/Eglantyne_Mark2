#include <Arduino.h>
#include <WiFi.h>

const char ssid[] = "Eglantyne";
const char pass[] = "Eglantyne";
const IPAddress ip(192, 168, 3, 11);
const IPAddress subnet(255, 255, 255, 0);
WiFiServer server(80);
WiFiClient client;

const unsigned long PING_INTERVAL = 10;
const unsigned long TIMEOUT = 2000;

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

        int values[3];
        sscanf(msg.c_str(), "%d,%d,%d", &values[0], &values[1], &values[2]);
        neopixelWrite(RGB_BUILTIN, values[0], values[1], values[2]);
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
