#include <WiFi.h>
#include <esp_now.h>
#include "msg.h"
#include "Robot.h"

void connection_init(Robot* r);
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len);

void Core0Task(void * parameter);
