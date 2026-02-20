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

// controller info handle
volatile ControlPacket global_control_pkt = {};

void setup(){
    neopixelWrite(RGB_BUILTIN, 255, 0, 0);

    // Serial
    Serial.begin(115200);
    Serial.println("Eglantyne Mark2 initializing...");
    delay(100);

    // SD
    sd.init();
    pinMode(SW, INPUT);

    // Eglantyne initializations
    krs1.begin();
    krs2.begin();
    Serial.println("Serials begin");

    // init robot
    Eglantyne.setSerial(&krs1, &krs2);
    Eglantyne.setLink();
    Serial.println("Eglantyne Mark2 prepared");

    Eglantyne.current();
    delay(10);
    Eglantyne.init_home(1);

    // sub loop initializations
    // connection_init(&Eglantyne);
    // lower_body_control_init(&Eglantyne);
    neopixelWrite(RGB_BUILTIN, 0, 0, 255);
    
    // esp now and upper body control task (core 0)
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
    // xTaskCreatePinnedToCore(
    //     Core1Task,
    //     "Core1Task",
    //     8192,
    //     NULL,
    //     configMAX_PRIORITIES+1, // max priority
    //     NULL,
    //     1 // core 1
    // );


    // test wake_up motion
    for (int i=0; i<9; i++){
        array<float, LINK_SIZE> motion = sd.read_motion("/motion1.txt", i);
        Eglantyne.move_all_t(motion, 1.5);
    }
    Eglantyne.init_home(2);
    delay(3000);
}

array<float, 3> foot2com_right = {-0.03, 0.05, 0.148};
array<float, 3> foot2com_left = {-0.03, -0.05, 0.148};
float theta = 0.0;
float height = 0.0;
float a = 0.0005;

bool set = false;
float t = 0.0;
void loop(){
    // array<float, 18> angles = Eglantyne.current();
    // Serial.println("Current angles:");
    // for (int i = 0; i < 18; i++){
    //     Serial.print(angles[i], 4); Serial.print(" ");
    // }
    // move legs with IK
    // foot2com_right [2] = 0.158 + height;
    // foot2com_left [2] = 0.158 + height;

    
    // fighting pose test
    // float right_elbow = global_control_pkt.arm_right[2];
    // float left_elbow = global_control_pkt.arm_left[2];

    // float twist = (right_elbow - left_elbow) / (120.0*PI/180.0);
    // if (twist >= 0){
    //     twist *= 10 * PI / 180.0f;
    // }else{
    //     twist += 0 * PI / 180.0f;
    // }
    // Eglantyne.move_leg_ik(foot2com_right, 30.0 * PI / 180.0 - twist, 0.0, true);
    // Eglantyne.move_leg_ik(foot2com_left, -30.0 * PI / 180.0 - twist, 0.0, false);

    // height += a;
    // if(height > 0){a = -0.0005;}
    // else if (height < -0.05){a = 0.0005;}
    
    Eglantyne.current();

    int sw = digitalRead(SW);
    // Serial.println(sw);
    if (sw == HIGH){
        t = millis();
        if (set){
            sd.write_motion("/motion1.txt", Eglantyne.current());
            set = false;
        }
    }
    if (sw == LOW) {
        set = true;
        if (millis() - t > 3000){
            sd.delete_motion_file("/motion1.txt");
        }
    }
    delay(100);
}
