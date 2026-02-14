#include <Arduino.h>
#include <vector>
#include <deque>
#include <mutex>
// my libs
#include "IcsHardSerialClass.h"
#include "Robot.h"
#include "pinassign.h"
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

// controller info handle
volatile ControlPacket global_control_pkt = {};

void setup(){
    neopixelWrite(RGB_BUILTIN, 255, 0, 0);

    Serial.begin(115200);
    Serial.println("Eglantyne Mark2 initializing...");
    delay(100);

    // Eglantyne initializations
    // krs1.begin();
    // krs2.begin();
    // Serial.println("Serials begin");

    // // init robot
    // Eglantyne.setSerial(&krs1, &krs2);
    // Eglantyne.setLink();
    // Serial.println("Eglantyne Mark2 prepared");

    // Eglantyne.current();
    // delay(100);
    // Eglantyne.init_home(1);
    
    neopixelWrite(RGB_BUILTIN, 0, 0, 255);

    // esp now and upper body control task (core 0)
    // connection_init(&Eglantyne);
    // xTaskCreatePinnedToCore(
    //     Core0Task,
    //     "Core0Task",
    //     8192,
    //     NULL,
    //     1,
    //     NULL,
    //     0 // core 0
    // );

    // lower body control task (core 1)
    lower_body_control_init(&Eglantyne);
    xTaskCreatePinnedToCore(
        Core1Task,
        "Core1Task",
        8192,
        NULL,
        configMAX_PRIORITIES+1, // max priority
        NULL,
        1 // core 1
    );
}

array<float, 3> foot2com_right = {0.0, 0.06, 0.118};
array<float, 3> foot2com_left = {0.0, -0.06, 0.118};
float theta = 0.0;
float a = 0.05;

void loop(){
    // move legs with IK
    // Eglantyne.move_leg_ik(foot2com_right, theta, 0.0, true);
    // Eglantyne.move_leg_ik(foot2com_left, theta, 0.0, false);

    // theta += a;
    // if(theta > 0.9){a = -0.05;}
    // else if (theta < -0.9){a = 0.05;}
    
    delay(100);
}
