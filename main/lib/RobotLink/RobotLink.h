#ifndef Link_h
#define Link_h

#include <Arduino.h>
#include "RobotLink.h"
#include "IcsHardSerialClass.h"

using namespace std;

#define FORWARD 1
#define REVERSE -1

class RobotLink{
public:
    RobotLink();

    void set_pos_ini(int pos_ini); // position 3500-11500
    void set_q_home(float q_home); //radian
    float getq_home();
    void set_limit(float q_min, float q_max); // radian
    void setInitialPositions(int pos_ini, float q_home, float q_min, float q_max);
    void setInitialPositionsDeg(int pos_ini, float q_home, float q_min, float q_max);
    float deg2rad(float deg);

    // new
    void setMotor(IcsHardSerialClass* motor, int motor_id, int dir);

    // controlling motor
    void move(float q_order);

    float getq_current(); // relax the motor and get pos(rad)
    int rad2pos(float rad);
    float pos2rad(int pos);

private:
    IcsHardSerialClass* motor;
    int motor_id;
    int dir;

    int pos_ini;
    float q_home;
    float q_max;
    float q_min;
};

#endif
