#include <Arduino.h>
#include <vector>
#include <deque>
#include <mutex>
// my libs
#include "IcsHardSerialClass.h"
#include "Robot.h"
#include "pinassign.h"
// SD card
#include "MotionSD.h"
// loops
#include "connection.h"
#include "lower_body.h"

// using
using std::array;
using std::vector;

// motor serial
#define BAUDRATE 1250000
#define TIMEOUT 1000
IcsHardSerialClass krs1(&Serial1, RobotEN1, BAUDRATE, TIMEOUT, RobotRX1, RobotTX1);
IcsHardSerialClass krs2(&Serial2, RobotEN2, BAUDRATE, TIMEOUT, RobotRX2, RobotTX2);

// robot control object
Robot Eglantyne;
// SD card
MotionSD sd;

// glbal variables
// controller peripheral variables
bool order_free = true;
bool connected = false;
volatile ControlPacket global_control_pkt = {};
// feedback variables
array<float, 2> com_x = {0.0f, 0.0f};
array<float, 3> arm_right_angles;
array<float, 3> arm_left_angles;

void setup(){
    neopixelWrite(RGB_BUILTIN, 255, 0, 0);

    // Serial
    Serial.begin(115200);
    Serial.println("Eglantyne Mark2 initializing...");
    delay(100);

    // Eglantyne initializations
    krs1.begin();
    krs2.begin();
    Serial.println("Serials begin");

    // init robot
    Eglantyne.setSerial(&krs1, &krs2);
    Eglantyne.setLink();
    // reinit home
    Eglantyne.set_leg_home_pose(0.045, HEIGHT_WALK);
    Serial.println("Eglantyne Mark2 prepared");

    array<float, LINK_SIZE> current = Eglantyne.current();
    delay(10);
    Eglantyne.move_all(current);
    Eglantyne.init_home(1);

    // sub loop initializations
    connection_init(&Eglantyne);

    // esp now and upper body control task (core 0)
    xTaskCreatePinnedToCore(
        Core0Task,
        "Core0Task",
        8192,
        NULL,
        1,
        NULL,
        0 // core 0
    );

    send_msg2controller("Eglantyne initializing...");
    lower_body_control_init(&Eglantyne, &sd);
    Serial.println("Eglantyne Mark2 ready");
    send_msg2controller("LOGO");
    neopixelWrite(RGB_BUILTIN, 0, 0, 255);
    
    order_free = false;
    // lower body control task (core 1)
    xTaskCreatePinnedToCore(
        Core1Task,
        "Core1Task",
        12288,
        NULL,
        configMAX_PRIORITIES-1, // max priority
        NULL,
        1 // core 1
    );
}

void loop(){
    vTaskDelay(10);
    return;
}
