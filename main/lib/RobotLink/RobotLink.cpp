#include "RobotLink.h"

RobotLink::RobotLink(){}

void RobotLink::set_pos_ini(int pos_ini){ this->pos_ini = pos_ini;}
void RobotLink::set_q_home(float q_home){this->q_home = q_home;}
float RobotLink::getq_home(){return this->q_home; }
void RobotLink::set_limit(float q_min, float q_max){this->q_min = q_min; this->q_max = q_max;}
void RobotLink::setInitialPositions(int pos_ini, float q_home, float q_min, float q_max){
    set_pos_ini(pos_ini);    // position
    set_q_home(q_home);      // radius
    set_limit(q_min, q_max); // radius
}
void RobotLink::setInitialPositionsDeg(int pos_ini, float q_home, float q_min, float q_max){
    set_pos_ini(pos_ini);                      // position
    set_q_home(deg2rad(q_home));               // degree
    set_limit(deg2rad(q_min), deg2rad(q_max)); // degree
}
float RobotLink::deg2rad(float deg){ return deg * 3.1415 / 180.0; }

// new
void RobotLink::setMotor(IcsHardSerialClass* motor, int motor_id, int dir){
    this->motor = motor;
    this->motor_id = motor_id;
    this->dir = dir;
}

// motor_control
// move motor to position (radian)
void RobotLink::move(float q_order){
    // if( q_order < q_min ){ q_order = q_min;
    // }else if( q_order > q_max ){ q_order = q_max; }
    motor->setPos(motor_id, rad2pos(q_order));
}
float RobotLink::getq_current(){
    return pos2rad(motor->setFree(motor_id));
}

int RobotLink::rad2pos(float rad){ 
    return (int) ( ( (float)dir * rad * 180.0 / M_PI ) * ( 8000.0 / 270 ) + (float)this->pos_ini ); 
}
float RobotLink::pos2rad(int pos){
    return (float)dir * (float)(pos - pos_ini) * 270.0/8000.0 * M_PI/180.0;
}
