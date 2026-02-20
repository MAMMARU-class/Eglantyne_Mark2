#include "connection.h"

uint8_t clientMac[6];
bool clientRegistered = false;

const unsigned long PING_INTERVAL = 1000;
const unsigned long TIMEOUT       = 3000;

unsigned long lastPing  = 0;
unsigned long lastReply = 0;

static Robot* robot;

void connection_init(Robot* r){
    robot = r;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (1);
    }

    esp_now_register_recv_cb(onReceive);

    Serial.println("ESP-NOW server ready");
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
}

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

    // size check
    if (len != sizeof(ControlPacket)) {
        Serial.print("Invalid packet size: ");
        Serial.println(len);
        return;
    }

    memcpy((void*)&global_control_pkt,
            data,
            sizeof(ControlPacket));

    // move robot arm
    std::array<float, 3> arm_right = {
        global_control_pkt.arm_right[0],
        global_control_pkt.arm_right[1],
        global_control_pkt.arm_right[2]
    };
    std::array<float, 3> arm_left = {
        global_control_pkt.arm_left[0],
        global_control_pkt.arm_left[1],
        global_control_pkt.arm_left[2]
    };

    // don't send any msg if free
    if (free){
        return 0;
    }else{
        // robot->move_arm_right(arm_right);
        // robot->move_arm_left(arm_left);
        robot->free_upper();
    }
}

// connection handling
void Core0Task(void * parameter){
    while(1){
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
        delay(10);
    }
}
