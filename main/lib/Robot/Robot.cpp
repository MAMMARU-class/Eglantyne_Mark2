#include "Robot.h"

Robot::Robot(){}

void Robot::setSerial(IcsHardSerialClass* serial1, IcsHardSerialClass* serial2){
    this->serial1 = serial1;
    this->serial2 = serial2;
}

array<float, LINK_SIZE> Robot::home(){
    array<float, LINK_SIZE> home;
    int link_num = 0;
    for(auto *link : link_set){
        home[link_num] = link->getq_home();
        link_num++;
    }
    return home;
}

array<float, LINK_SIZE> Robot::current(){
    array<float, LINK_SIZE> current;
    int link_num = 0;
    for(auto *link : link_set){
        current[link_num] = link->getq_current();
        link_num++;
    }
    return current;
}
float Robot::current_link(int id){
    return link_set[id]->getq_current();
}

void Robot::init_home(float t){
    array<float, LINK_SIZE> current = this->current();
    array<float, LINK_SIZE> home = this->home();

    array<float, LINK_SIZE> diff;
    for(int id=0; id<LINK_SIZE; id++){
        diff[id] = home[id] - current[id];
    }

    // this->move_all(home);
    
    int step = int(t/CTRL_CYCLE * 1000);
    for(int i=0; i<=step; i++){
        array<float, LINK_SIZE> motion;
        for(int id=0; id<LINK_SIZE; id++){
            motion[id] = current[id] + diff[id]*( (float)(i) ) / (float)(step);
        }
        this->move_all(motion);
        delay(CTRL_CYCLE);
    }
}

void Robot::move_all(array<float, LINK_SIZE> motion){
    for(int id=0; id<LINK_SIZE; id++){
        link_set[id]->move(motion[id]);
    }
}
void Robot::move_link(int id, float q_order){
        link_set[id]->move(q_order);
}

void Robot::move_arm_right(array<float, 3> motion){
    for(int id=0; id<3; id++){
        link_set[id+0]->move(motion[id]);
    }
}
void Robot::move_arm_left(array<float, 3> motion){
    for(int id=0; id<3; id++){
        link_set[id+3]->move(motion[id]);
    }
}
void Robot::move_leg_right(array<float, 6> motion){
    for(int id=0; id<6; id++){
        link_set[id+6]->move(motion[id]);
    }
}
void Robot::move_leg_left(array<float, 6> motion){
    for(int id=0; id<6; id++){
        link_set[id+12]->move(motion[id]);
    }
}

void Robot::move_leg_ik(array<float, 3> foot2com, float theta, float phi, bool is_right){
    array<float, 6> angles;
    if(phi == 0.0f){
        angles = this->leg_ik_solver_phi_zero(foot2com, theta, is_right);
    }else{
        angles = {0};
    }

    if (is_right){ this->move_leg_right(angles);
    }else{ this->move_leg_left(angles); }
}
