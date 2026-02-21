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
volatile ControlPacket global_control_pkt = {};
bool order_free = true;
bool connected = false;

// motionn register
bool motion_register_mode = false;

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
    Serial.println("Eglantyne Mark2 prepared");

    Eglantyne.current();
    delay(10);
    Eglantyne.init_home(1);

    // sub loop initializations
    connection_init(&Eglantyne);
    lower_body_control_init(&Eglantyne, &sd);
    neopixelWrite(RGB_BUILTIN, 0, 0, 255);

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

    // init register mode
    pinMode(SW, INPUT);
    if (digitalRead(SW) == LOW){
        order_free = true;
        Serial.println("motion register mode");
        neopixelWrite(RGB_BUILTIN, 255, 192, 203);
        motion_register_mode = true;
        return;
    }
    
    if(!motion_register_mode){
        order_free = false;
        // lower body control task (core 1)
        xTaskCreatePinnedToCore(
            Core1Task,
            "Core1Task",
            12288,
            NULL,
            configMAX_PRIORITIES+1, // max priority
            NULL,
            1 // core 1
        );
    }
}

void loop(){
    // sleep if not in motion register mode
    if (!motion_register_mode){
        vTaskDelay(portMAX_DELAY);
        return;
    }

    // choose motion file to register
    size_t file_id = 0;
    std::string fname;
    while(1){
        // choose
        if (global_control_pkt.button_right[1] == 0){
            file_id++;
            while(global_control_pkt.button_right[1] == 0){
                delay(10);
            }
            delay(100);
        }
        send_msg2controller(sd.get_filename_by_id(file_id).c_str());
        delay(100);

        // select
        if (global_control_pkt.button_right[0] == 0){
            fname = sd.get_filename_by_id(file_id);
             Serial.print("Selected file: ");
             Serial.println(fname.c_str());
             while(global_control_pkt.button_right[0] == 0){
                delay(10);
            }
            delay(100);
            Eglantyne.free_all();
            neopixelWrite(RGB_BUILTIN, 0, 0, 255);
            send_msg2controller("LOGO");
            break;
        }
    }

    bool set = false;
    while(1){
        // set motion
        if (set && global_control_pkt.button_left[1] == 1){
            sd.write_motion(fname.c_str(), Eglantyne.current());
            set = false;
        }

        if (global_control_pkt.button_left[1] == 0){
            Serial.println("Registering motion...");
            set = true;
            while(global_control_pkt.button_left[1] == 0){
                delay(10);
            }
        }

        // delete motion
        if (global_control_pkt.button_left[2] == 0){
            sd.delete_motion_file(fname.c_str());
            neopixelWrite(RGB_BUILTIN, 255, 0, 0);
            Serial.println("Motion deleted");
            while(global_control_pkt.button_left[2] == 0){
                delay(10);
            }
            delay(100);
            neopixelWrite(RGB_BUILTIN, 0, 0, 255);
            return;
        }

        // play motion
        if (global_control_pkt.button_left[0] == 0){
            neopixelWrite(RGB_BUILTIN, 0, 255, 0);

            Eglantyne.init_home(1);
            sd.play_motion(&Eglantyne, fname.c_str(), 0.5);
            Eglantyne.init_home(1);

            while(global_control_pkt.button_left[0] == 0){
                delay(10);
            }

            delay(100);
            neopixelWrite(RGB_BUILTIN, 0, 0, 255);
        }

       delay(100);
    }
}
