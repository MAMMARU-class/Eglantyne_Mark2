#include "Robot.h"

Robot::Robot(){}

// initialization
void Robot::setSerial(IcsHardSerialClass* serial1, IcsHardSerialClass* serial2){
    this->serial1 = serial1;
    this->serial2 = serial2;
}

void Robot::init_home(float t){
    delay(10);
    array<float, LINK_SIZE> current = this->current();
    this->move_all(current);
    array<float, LINK_SIZE> home = this->home();

    array<float, LINK_SIZE> diff;
    for(int id=0; id<LINK_SIZE; id++){
        diff[id] = home[id] - current[id];
    }
    
    int step = int(t/CTRL_CYCLE * 1000);
    for(int i=0; i<=step; i++){
        array<float, LINK_SIZE> motion;
        for(int id=0; id<LINK_SIZE; id++){
            motion[id] = current[id] + diff[id]*( (float)(i) ) / (float)(step);
        }
        delay(CTRL_CYCLE);
        this->move_all(motion);
    }
}

void Robot::init_home_arm(float t){
    array<float, LINK_SIZE> home = this->home();
    array<float, 3> arm_home_right = {home[0], home[1], home[2]};
    array<float, 3> arm_home_left  = {home[3], home[4], home[5]};
    this->move_arm_t(arm_home_right, arm_home_left, t);
}

// set and get robot home
void Robot::set_leg_home_pose(float leg_dist, float height){
    array<float, 6> angles_right = 
        this->leg_ik_solver_phi_zero({0, leg_dist, height}, 0, true);
    array<float, 6> angles_left = 
        this->leg_ik_solver_phi_zero({0, -leg_dist, height}, 0, false);
    
    for(int id=0; id<6; id++){
        link_set[id+6]->set_q_home(angles_right[id]);
        link_set[id+12]->set_q_home(angles_left[id]);
    }
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

// get motor positions (radian)
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

// move motors (radian)
void Robot::move_all(array<float, LINK_SIZE> motion){
    for(int id=0; id<LINK_SIZE; id++){
        link_set[id]->move(motion[id]);
    }
}

void Robot::move_all_t(array<float, LINK_SIZE> goal, float t){
    delay(10);
    array<float, LINK_SIZE> current = this->current();
    this->move_all(current);

    array<float, LINK_SIZE> diff;
    for(int id=0; id<LINK_SIZE; id++){
        diff[id] = goal[id] - current[id];
    }
    
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

void Robot::move_link(int id, float q_order){
        link_set[id]->move(q_order);
}
void Robot::move_link_t(int id, float q_order, float t){
    float current = this->current_link(id);
    float diff = q_order - current;

    int step = int(t/CTRL_CYCLE * 1000);
    for(int i=0; i<=step; i++){
        float motion = current + diff*( (float)(i) ) / (float)(step);
        this->move_link(id, motion);
        delay(CTRL_CYCLE);
    }
}

void Robot::move_arm_right(array<float, 3> motion){
    arm_right_angles[0] = motion[0];
    arm_right_angles[1] = motion[1];
    arm_right_angles[2] = motion[2];
    for(int id=0; id<3; id++){
        link_set[id+0]->move(motion[id]);
    }
}
void Robot::move_arm_left(array<float, 3> motion){
    arm_left_angles[0] = motion[0];
    arm_left_angles[1] = motion[1];
    arm_left_angles[2] = motion[2];
    for(int id=0; id<3; id++){
        link_set[id+3]->move(motion[id]);
    }
}
void Robot::move_arm_t(array<float, 3> right_motion, array<float, 3> left_motion, float t){
    array<float, 3> current_right;
    array<float, 3> current_left;
    for(int id=0; id<3; id++){
        current_right[id] = link_set[id+0]->getq_current();
        current_left[id] = link_set[id+3]->getq_current();
    }

    array<float, 3> diff_right;
    array<float, 3> diff_left;
    for(int id=0; id<3; id++){
        diff_right[id] = right_motion[id] - current_right[id];
        diff_left[id] = left_motion[id] - current_left[id];
    }

    int step = int(t/CTRL_CYCLE * 1000);
    for(int i=0; i<=step; i++){
        array<float, 3> motion_right;
        array<float, 3> motion_left;
        for(int id=0; id<3; id++){
            motion_right[id] = current_right[id] + diff_right[id]*( (float)(i) ) / (float)(step);
            motion_left[id] = current_left[id] + diff_left[id]*( (float)(i) ) / (float)(step);
        }
        this->move_arm_right(motion_right);
        this->move_arm_left(motion_left);
        delay(CTRL_CYCLE);
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
void Robot::move_leg_t(array<float, 6> right_motion, array<float, 6> left_motion, float t){
    array<float, 6> current_right;
    array<float, 6> current_left;
    for(int id=0; id<6; id++){
        current_right[id] = link_set[id+6]->getq_current();
        current_left[id] = link_set[id+12]->getq_current();
    }

    array<float, 6> diff_right;
    array<float, 6> diff_left;
    for(int id=0; id<6; id++){
        diff_right[id] = right_motion[id] - current_right[id];
        diff_left[id] = left_motion[id] - current_left[id];
    }

    int step = int(t/CTRL_CYCLE * 1000);
    for(int i=0; i<=step; i++){
        array<float, 6> motion_right;
        array<float, 6> motion_left;
        for(int id=0; id<6; id++){
            motion_right[id] = current_right[id] + diff_right[id]*( (float)(i) ) / (float)(step);
            motion_left[id] = current_left[id] + diff_left[id]*( (float)(i) ) / (float)(step);
        }
        this->move_leg_right(motion_right);
        this->move_leg_left(motion_left);
        delay(CTRL_CYCLE);
    }
}

void Robot::move_safely_fall(
    array<float, 3> current_order_right,
    float current_theta_right,
    array<float, 3> current_order_left,
    float current_theta_left,
    float height,
    float t
){
    array<float, 3> diff_right = {0.0f - current_order_right[0],  0.04f - current_order_right[1], height - current_order_right[2]};
    array<float, 3> diff_left =  {0.0f - current_order_left[0] , -0.04f - current_order_left[1] , height - current_order_left[2]};
    float diff_theta_right = 0.0f - current_theta_right;
    float diff_theta_left = 0.0f - current_theta_left;

    int step = int(t/CTRL_CYCLE * 1000);
    for(int i=0; i<=step; i++){
        array<float, 3> motion_right;
        array<float, 3> motion_left;

        for(int id = 0; id<3; id++){
            motion_right[id] = current_order_right[id] + diff_right[id]*( (float)(i) ) / (step);
            motion_left[id]  = current_order_left[id]  + diff_left[id] *( (float)(i) ) / (step);
        }

        this->move_leg_ik(motion_right, current_theta_right + diff_theta_right * ( (float)(i) ) / (step), 0.0, 0.0, true);
        this->move_leg_ik(motion_left,  current_theta_left  + diff_theta_left  * ( (float)(i) ) / (step), 0.0, 0.0, false);

        delay(CTRL_CYCLE);
    }
}

// calculation
void Robot::move_leg_ik(array<float, 3> foot2com, float theta, float phi_upper, float phi_lower, bool is_right){
    array<float, 6> angles = this->leg_ik_solver_phi_zero(foot2com, theta, is_right);
    angles[2] += phi_upper;
    angles[4] -= phi_lower;

    if (is_right){ this->move_leg_right(angles);
    }else{ this->move_leg_left(angles); }
}

void Robot::move_leg_ik_t(
    array<float, 3> foot2com_r, float theta_r, float phi_upper_r, float phi_lower_r,
    array<float, 3> foot2com_l, float theta_l, float phi_upper_l, float phi_lower_l,
    float t)
{
    array<float, LINK_SIZE> current = this->current();
    array<float, 6> leg_right_current;
    array<float, 6> leg_left_current;

    leg_right_current = {current[6], current[7], current[8], current[9], current[10], current[11]};
    leg_left_current  = {current[12], current[13], current[14], current[15], current[16], current[17]};

    array<float, 6> leg_right_goal = this->leg_ik_solver_phi_zero(foot2com_r, theta_r, true);
    leg_right_goal[2] += phi_upper_r;
    leg_right_goal[4] -= phi_lower_r;
    array<float, 6> leg_left_goal  = this->leg_ik_solver_phi_zero(foot2com_l, theta_l, false);
    leg_left_goal[2] += phi_upper_l;
    leg_left_goal[4] -= phi_lower_l;

    array<float, 6> diff_right;
    array<float, 6> diff_left;
    for(int id=0; id<6; id++){
        diff_right[id] = leg_right_goal[id] - leg_right_current[id];
        diff_left[id]  = leg_left_goal[id]  - leg_left_current[id];
    }

    int step = int(t/CTRL_CYCLE * 1000);
    for(int i=0; i<=step; i++){
        array<float, 6> motion_right;
        array<float, 6> motion_left;
        for(int id=0; id<6; id++){
            motion_right[id] = leg_right_current[id] + diff_right[id]*( (float)(i) ) / (float)(step);
            motion_left[id]  = leg_left_current[id]  + diff_left[id] *( (float)(i) ) / (float)(step);
        }
        this->move_leg_right(motion_right);
        this->move_leg_left(motion_left);
        delay(CTRL_CYCLE);
    }
}

// free motors
void Robot::free_upper(){
    for(int id=0; id<6; id++){
        link_set[id]->getq_current();
    }
}
void Robot::free_all(){
    for(int id=0; id<LINK_SIZE; id++){
        link_set[id]->getq_current();
    }
}
