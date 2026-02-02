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
    // array<float, LINK_SIZE> current;
    // array<float, LINK_SIZE> home;
    // vector< array<float, LINK_SIZE> > home_motion_list;

    // int link_num = 0;
    // for(auto* link : link_set){
    //     current[link_num] = link->getq_current();
    //     home[link_num] = link->getq_home();
    //     link_num++;
    // }
    
    // int step_end = t*1000 / CONTROL_CYCLE;
    // for(int step=0; step<step_end; step++){
    //     array<float, 18> motion;
    //     for(int id=0; id<current.size(); id++){
    //         motion[id] = ( current[id]*( (float)(step_end-step) ) + home[id]*( (float)(step) ) ) / (float)step_end;
    //     }
    //     home_motion_list.push_back(motion);
    // }
    // for(array<float, 18> motion : home_motion_list){
    //     move_all(motion);
    //     delay(CONTROL_CYCLE);
    // }
}

void Robot::move_all(array<float, LINK_SIZE> motion){
    for(int id=0; id<LINK_SIZE; id++){
        link_set[id]->move(motion[id]);
    }
}
void Robot::move_link(int id, float q_order){
        link_set[id]->move(q_order);
}
