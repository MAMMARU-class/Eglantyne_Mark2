#ifndef CONNECTION_H
#define CONNECTION_H

#include <WiFi.h>
#include <esp_now.h>
#include "msg.h"
#include "Robot.h"
#include "pinassign.h"

void connection_init(Robot* r);
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len);

void Core0Task(void * parameter);
void send_msg2controller(const char* msg);

#endif
