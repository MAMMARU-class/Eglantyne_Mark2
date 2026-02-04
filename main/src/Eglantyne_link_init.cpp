#include "Robot.h"

void Robot::setLink(){
    arm_pitch_right.setInitialPositions(9470, 0.0 ,0,0);
    arm_pitch_right.setMotor(serial1, 1, REVERSE);

    arm_roll_right.setInitialPositions(10420, 30.0 * 3.14/180 ,0,0);
    arm_roll_right.setMotor(serial1, 2, REVERSE);

    hand_right.setInitialPositions(5700, 0.0 ,0,0);
    hand_right.setMotor(serial1, 3, FORWARD);


    arm_pitch_left.setInitialPositions(5930, 0.0, 0,0);
    arm_pitch_left.setMotor(serial1, 4, FORWARD);

    arm_roll_left.setInitialPositions(10300, 30.0 * 3.14/180, 0,0);
    arm_roll_left.setMotor(serial1, 5, REVERSE);

    hand_left.setInitialPositions(6350, 0.0 ,0,0);
    hand_left.setMotor(serial1, 6, FORWARD);


    leg_yaw_right.setInitialPositions(7380, 0.0 ,0,0);
    leg_yaw_right.setMotor(serial2, 1, REVERSE);

    leg_roll_right.setInitialPositions(9000, 0.0, 0,0);
    leg_roll_right.setMotor(serial2, 2, REVERSE);

    leg_upper_right.setInitialPositions(9850, 0.7953, 0,0);
    leg_upper_right.setMotor(serial2, 3, REVERSE);

    leg_under_right.setInitialPositions(4200, 1.8852, 0,0);
    leg_under_right.setMotor(serial2, 4, FORWARD);

    foot_pitch_right.setInitialPositions(5890, 0.4611, 0,0);
    foot_pitch_right.setMotor(serial2, 5, FORWARD);

    foot_roll_right.setInitialPositions(7600, 0.0, 0,0);
    foot_roll_right.setMotor(serial2, 6, REVERSE);


    leg_yaw_left.setInitialPositions(7500, 0.0, 0,0);
    leg_yaw_left.setMotor(serial2, 7, REVERSE);

    leg_roll_left.setInitialPositions(8330, 0.0, 0,0);
    leg_roll_left.setMotor(serial2, 8, FORWARD);

    leg_upper_left.setInitialPositions(9750, 0.5236, 0,0);
    leg_upper_left.setMotor(serial2, 9, REVERSE);

    leg_under_left.setInitialPositions(4320, 1.0472, 0,0);
    leg_under_left.setMotor(serial2, 10, FORWARD);

    foot_pitch_left.setInitialPositions(6090, 0.5236, 0,0);
    foot_pitch_left.setMotor(serial2, 11, FORWARD);

    foot_roll_left.setInitialPositions(7400, 0.0 ,0,0);
    foot_roll_left.setMotor(serial2, 12, REVERSE);

    // this->link_set = {&arm_pitch_right}; // for board test

    // this->link_set = {
    //     &arm_pitch_right, &arm_roll_right, &hand_right,
    //     &arm_pitch_left, &arm_roll_left, &hand_left };
    
    this->link_set = {
        &arm_pitch_right, &arm_roll_right, &hand_right,
        &arm_pitch_left, &arm_roll_left, &hand_left,
        &leg_yaw_right, &leg_roll_right, &leg_upper_right, &leg_under_right, &foot_pitch_right, &foot_roll_right,
        &leg_yaw_left, &leg_roll_left, &leg_upper_left, &leg_under_left, &foot_pitch_left, &foot_roll_left
    };
}
