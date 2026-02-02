#ifndef robot_h
#define robot_h

#include <Arduino.h>
#include <vector>
#include <string>
#include "RobotLink.h"

#define CONTROL_CYCLE 10 // ms
// for board test
#define LINK_SIZE 1
// #define LINK_SIZE 18
using std::vector;
using std::array;

class Robot{
public:
    Robot();
    // initialization
    void setSerial(IcsHardSerialClass* serial1, IcsHardSerialClass* serial2);
    void setLink();
    void init_home(float t);

    // get robot info
    array<float, LINK_SIZE> home();

    // get motor positions (radian)
    array<float, LINK_SIZE> current();
    float current_link(int id);

    // move motors (radian)
    void move_all(array<float, LINK_SIZE> motion);
    void move_link(int id, float q_order);

private:
    // serial
    IcsHardSerialClass* serial1;
    IcsHardSerialClass* serial2;

    // link object
    array<RobotLink*, LINK_SIZE> link_set;

    RobotLink arm_pitch_right;
    RobotLink arm_roll_right;
    RobotLink hand_right;

    RobotLink arm_pitch_left;
    RobotLink arm_roll_left;
    RobotLink hand_left;

    RobotLink leg_yaw_right;
    RobotLink leg_roll_right;
    RobotLink leg_upper_right;
    RobotLink leg_under_right;
    RobotLink foot_pitch_right;
    RobotLink foot_roll_right;

    RobotLink leg_yaw_left;
    RobotLink leg_roll_left;
    RobotLink leg_upper_left;
    RobotLink leg_under_left;
    RobotLink foot_pitch_left;
    RobotLink foot_roll_left;
};

#endif
