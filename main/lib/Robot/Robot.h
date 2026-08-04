#ifndef robot_h
#define robot_h

#include <Arduino.h>
#include <vector>
#include <string>
#include "RobotLink.h"
#include "msg.h"

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
    void init_home_arm(float t);

    // set and get robot home
    void set_leg_home_pose(float leg_dist, float height);
    array<float, LINK_SIZE> home();

    // get motor positions (radian)
    array<float, LINK_SIZE> current();
    float current_link(int id);

    // move motors (radian)
    void move_all(array<float, LINK_SIZE> motion);
    void move_all_t(array<float, LINK_SIZE> goal, float t);
    void move_link(int id, float q_order);
    void move_link_t(int id, float q_order, float t);

    void move_arm_right(array<float, 3> motion);
    void move_arm_left(array<float, 3> motion);
    void move_arm_t(array<float, 3> right_motion, array<float, 3> left_motion, float t);
    void move_leg_right(array<float, 6> motion);
    void move_leg_left(array<float, 6> motion);
    void move_leg_t(array<float, 6> right_motion, array<float, 6> left_motion, float t);

    void move_safely_fall(
        array<float, 3> current_order_right,
        float current_theta_right,
        array<float, 3> current_order_left,
        float current_theta_left,
        float height,
        float t
    );

    // calculation
    void move_leg_ik(
        array<float, 3> foot2com, float theta, 
        float phi_upper, float phi_lower,
        bool is_right
    );
    void move_leg_ik_t(
        array<float, 3> foot2com_r, float theta_r, float phi_upper_r, float phi_lower_r,
        array<float, 3> foot2com_l, float theta_l, float phi_upper_l, float phi_lower_l,
        float t
    );

    // free motors
    void free_upper();
    void free_all();

    // Kinematics (foot2com: (x,y,z)[m], theta: foot_rotation[rad], is_right: bool)
    array<float, 6> leg_ik_solver_phi_zero(array<float, 3> foot2com, float theta, bool is_right);
    array<float, 3> arm_k_solver(array<float, 3> arm_angles);

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
    float l_foot_x = 23.0;

    float l_arm_upper = 82.84;
    float l_arm_lower = 55;

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

    array<array<float, 4>, 4> mul_T_matrices(array<array<float, 4>, 4> mat1, array<array<float, 4>, 4> mat2);
};

#endif
