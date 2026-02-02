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
// #define BAUDRATE 1250000
// for board test
#define BAUDRATE 115200
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
}

double angle = 0;
array<float, LINK_SIZE> pos;
array<float, LINK_SIZE> current;

void loop(){
    pos[0] = 0.0;
    Eglantyne.move_all(pos);
    delay(1000);
    current = Eglantyne.current();
    Serial.print("Current pos: ");
    Serial.println(current[0]);

    pos[0] = 1.2;
    Eglantyne.move_all(pos);
    delay(1000);
    current = Eglantyne.current();
    Serial.print("Current pos: ");
    Serial.println(current[0]);
}
