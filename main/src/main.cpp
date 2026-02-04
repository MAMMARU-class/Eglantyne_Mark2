#include <Arduino.h>
#include <vector>
#include <deque>
#include <mutex>
// my libs
#include "IcsHardSerialClass.h"
#include "Robot.h"
#include "pinassign.h"

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

void setup(){
    neopixelWrite(RGB_BUILTIN, 255, 0, 0);

    Serial.begin(115200);
    Serial.println("Eglantyne Mark2 initializing...");
    delay(100);

    krs1.begin();
    krs2.begin();
    Serial.println("Serials begin");

    // init robot
    Eglantyne.setSerial(&krs1, &krs2);
    Eglantyne.setLink();
    Serial.println("Eglantyne Mark2 prepared");

    neopixelWrite(RGB_BUILTIN, 0, 255, 0);

    array<float, LINK_SIZE> home = Eglantyne.home();
    Serial.print("Home pos: ");
    Serial.println(home[0]);
    array<float, LINK_SIZE> current = Eglantyne.current();
    Serial.print("Current pos: ");
    Serial.println(current[0]);
    
    Eglantyne.init_home(2);
    delay(1000);
    

    neopixelWrite(RGB_BUILTIN, 0, 0, 255);
}

array<float, 3> foot2com_right = {0.0, 0.03, 0.158};
array<float, 3> foot2com_left = {0.0, -0.03, 0.158};
float theta = 0.0;

float x_max = 0.03;
float x_min = -0.03;

float y_max = 0.02;
float y_min = 0.0;

float z_max = 0.158;
float z_min = 0.138;

float theta_max = 1.4;
float theta_min = -0.2;

array<float, 6> leg_motion;
int step = 100;

void loop(){
    for(int i = 0; i<step; i++){
        foot2com_right[0] = x_min + ( (x_max - x_min) * (float)(i) / (float)(step) );
        foot2com_left[0]  = x_max - ( (x_max - x_min) * (float)(i) / (float)(step) );

        foot2com_right[1] = 0.03 + y_min + ( (y_max - y_min) * (float)(i) / (float)(step) );
        foot2com_left[1] = -0.03 - (y_min + ( (y_max - y_min) * (float)(i) / (float)(step) ));

        // foot2com_right[2] = z_max - ( (z_max - z_min) * (float)(i) / (float)(step) );
        // foot2com_left[2] = z_max - ( (z_max - z_min) * (float)(i) / (float)(step) );

        // theta = theta_min + ( (theta_max - theta_min) * (float)(i) / (float)(step) );

        Eglantyne.move_leg_ik(foot2com_right, theta, 0.0, true);
        Eglantyne.move_leg_ik(foot2com_left, -theta, 0.0, false);
        delay(10);
    }
    for(int i = 0; i<step; i++){
        foot2com_right[0] = x_max - ( (x_max - x_min) * (float)(i) / (float)(step) );
        foot2com_left[0]  = x_min + ( (x_max - x_min) * (float)(i) / (float)(step) );

        foot2com_right[1] = 0.03 + y_max - ( (y_max - y_min) * (float)(i) / (float)(step) );
        foot2com_left[1] = -0.03 - (y_max - ( (y_max - y_min) * (float)(i) / (float)(step) ));

        // foot2com_right[2] = z_min + ( (z_max - z_min) * (float)(i) / (float)(step) );
        // foot2com_left[2] = z_min + ( (z_max - z_min) * (float)(i) / (float)(step) );

        // theta = theta_max - ( (theta_max - theta_min) * (float)(i) / (float)(step) );

        Eglantyne.move_leg_ik(foot2com_right, theta, 0.0, true);
        Eglantyne.move_leg_ik(foot2com_left, -theta, 0.0, false);
        delay(10);
    }

    // foot2com_right[2] = 0.138;
    // leg_motion = Eglantyne.leg_ik_solver_phi_zero(foot2com_right, 0.0, true);
    // Serial.print("Leg right IK: ");
    // for(int i=0; i<6; i++){
    //     Serial.print(leg_motion[i]);
    //     Serial.print(", ");
    // }
    // Serial.println("");
    // delay(1000);
}
