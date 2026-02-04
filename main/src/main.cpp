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

array<float, 3> foot2com_right = {0.05, 0.04, 0.138};
array<float, 3> foot2com_left = {0.0, -0.03, 0.138};
float theta = 0.0;
float a = 0.001;

void loop(){
    // move legs with IK
    Eglantyne.move_leg_ik(foot2com_right, theta, 0.0, true);

    theta += a;
    if(theta > 0.2){a = -0.001;}
    else if (theta < -0.2){a = 0.001;}

    delay(10);
}
