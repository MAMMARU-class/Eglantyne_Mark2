#ifndef robot_h
#define robot_h

#include <Arduino.h>
#include <vector>
#include <string>
#include "RobotLink.h"

#define CTRL_CYCLE 10 // ms
// #define LINK_SIZE 1 // for board test
#define LINK_SIZE 18
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

    void move_arm_right(array<float, 3> motion);
    void move_arm_left(array<float, 3> motion);
    void move_leg_right(array<float, 6> motion);
    void move_leg_left(array<float, 6> motion);

    // calculation
    void move_leg_ik(array<float, 3> foot2com, float theta, float phi, bool is_right);

    // Inverse Kinematics (foot2com: (x,y,z)[m], theta: foot_rotation[rad], is_right: bool)
    array<float, 6> leg_ik_solver_phi_zero(array<float, 3> foot2com, float theta, bool is_right);

private:
    // link length [mm]
    float l_pivot2com = 70;
    float l_com_z = -52.2;
    float l_com_y = 30;

    float l_base_roll = 22.2;
    float l_roll2pitch = 26.01;
    float l_leg = 78.02;

    float l_roll_com = l_com_z + l_base_roll;

    float l_foot_z = 37.5;
    float l_foot_x = 8.0;

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
