#include "lower_body.h"

static array<float, 3> vd = {0.0001f, 0.0f, 0.0f};

static Mode mode = Mode::WALK;

static Phase phase = Phase::START;
static Phase phase_next;

// step counter
static int phase_length;
static int phase_count;

// other classees
static Robot* robot;
static MotionSD* sd;
GaitController controller;
SensorFB sensor;

void lower_body_control_init(Robot* r, MotionSD* s){
    robot = r;
    sd = s;

    sd->init();
    pinMode(SW, INPUT);

    sensor.init();
    delay(1000);
    sensor.update();
    delay(1000);
    Serial.println("Lower body control initialized");
}

array<float, 3> update_vel(array<float, 3> vd){
    return vd;
}

void init_phase(Mode next_mode, Phase next_phase, float next_phase_time){
    mode = next_mode;
    phase = next_phase;
    phase_length = next_phase_time * CTRL_STEP;
    phase_count = 0;
}

Phase update_phase(){
    return Phase::DOUBLE;
}

void Core1Task(void * parameter){
    // check is robot and sd is given
    if(robot == nullptr){
        Serial.println("Robot is null");
        while(1);
    }
    if(sd == nullptr){
        Serial.println("MotionSD is null");
        while(1);
    }

    while(1) {
        /* #########################################################################
        ORDER AND MODE INITIALIZEAITON
        In the first step, check
        - Torque off order is given -> free all joint and skip the rest of the loop
        - Fall                      -> Switch to FALL phase
        - Velocity update           -> Update vd with controller input and sensor feedback. Switch to WALK mode if vd is large enough
        - Mode                      -> If in WAIT mode, skip the rest of the loop
        ##########################################################################*/
        // torque off order
        if(global_control_pkt.button_right[0] == 0 && global_control_pkt.button_left[0] == 0){
            order_free = true;
            robot->free_all();
            continue;
        }
        // fall check
        if(sensor.fall() && phase != Phase::FALL && phase != Phase::WAKE){
            phase = Phase::FALL;
            mode  = Mode::WALK;
        }

        // update vd
        vd = update_vel(vd);
        // array<float, 3> vd_fb = sensor.vd_fb(vd);
        // vd[0] += vd_fb[0];
        // vd[1] += vd_fb[1];
        // vd[2] += vd_fb[2];

        // walk if vd is large enough
        if (abs(vd[0]) > 0.005f || abs(vd[1]) > 0.005f || abs(vd[2]) > 0.005f){
            mode = Mode::WALK;
        }

        // do nothing if WAIT
        if (mode == Mode::WAIT){
            continue;
        }

        /* #########################################################################
        TRAJECTORY CALCULATION AND PHASE UPDATE
        In the second step, calculate the desired com position based on the phase. Update the phase at the end of each phase duration
        For each phase, 
        - START    : Initialize satrt parameters and phase length. Next phase is SINGLE 
        - END      : Initialize end parameters and phase length. Next phase is START, and change mode to WAIT
        - SINGLE   : At the middle of the phase, decide next phase and next foot position. Next phase is (DOUBLE / END / )
        - DOUBLE   : CoM transition between SINGLE and SINGLE. calculate next SINGLE phase parameters and change pivot in the first step. Next phase is SINGLE.
        - FLIGHT   : 
        - FALL     : After slip is detected, free upper body and shrink lower body for the safety. Next phase is WAKE.
        - WAKE     : WAKE the robot up. Next phase is START. and change the mode to WAIT.
        ##########################################################################*/
        // move robot
        // init com_pos
        array<array<float, 5>, 3> com_pos = {{
            {0.0,  0.04, 0.158, 0.0, 0.0},
            {0.0, -0.04, 0.158, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0}
        }};
        switch (phase){
            case Phase::START:{
                if (phase_count == 0){
                    Serial.println("phase: START");
                    controller.inverse_pivot();
                    controller.init_param_walk();
                    controller.init_start();
                    float T_ds = controller.get_T_sup() * controller.get_ds_ratio() * 0.5f;
                    phase_length = T_ds * CTRL_STEP;
                }
                com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP);
                
                // phase transition
                phase_count++;
                if (phase_count == phase_length){
                    init_phase(
                        Mode::WALK, 
                        Phase::SINGLE, 
                        controller.get_T_sup()*(1.0f - controller.get_ds_ratio())
                    );
                }
                break;
            }
            
            case Phase::END:{
                if (phase_count == 0){
                    Serial.println("phase: END");
                    controller.inverse_pivot();
                    controller.init_end();
                    float T_ds = controller.get_T_sup() * controller.get_ds_ratio() * 0.5f;
                    phase_length = T_ds * CTRL_STEP;
                }
                com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP);

                // phase transition
                phase_count++;
                if (phase_count == phase_length){
                    init_phase(
                        Mode::WAIT,
                        Phase::START,
                        0
                    );
                }
                break;
            }

            case Phase::SINGLE:{
                if (phase_count == 0){
                    Serial.println("phase: SINGLE");
                    controller.init_single_0();
                }
                if (phase_count == int(phase_length/2)){             
                    // sensor feedback
                    // array<float, 2> foot_pos_fb = sensor.foot_pos_fb();
                    // sensor.init_norm();
                    array<float, 2> foot_pos_fb = {0,0};
                    // update state variables in gait controller
                    controller.update_state_variables(vd, foot_pos_fb);
                    controller.init_single_half();
                    phase_next = update_phase();
                }
                com_pos = controller.calc_com_traj_single(phase_count / (float)CTRL_STEP);

                // phase transition
                phase_count++;
                if (phase_count == phase_length){
                    init_phase(
                        Mode::WALK, 
                        phase_next,
                        controller.get_T_ds()
                    );
                }
                break;
            }

            case Phase::DOUBLE:{
                if (phase_count == 0){
                    Serial.println("phase: DOUBLE");
                    controller.inverse_pivot();
                    controller.init_state_variables();
                }
                com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP);
                
                // phase transition
                phase_count++;
                if (phase_count == phase_length){
                    init_phase(
                        Mode::WALK,
                        Phase::SINGLE, 
                        controller.get_T_sup()*(1.0f - controller.get_ds_ratio())
                    );
                }
                break;
            }

            case Phase::FLIGHT:{
                break;
            }

            case Phase::FALL:{
                order_free = true;
                robot->free_upper();
                array<float, 3> current_order_right = {com_pos[0][0], com_pos[0][1], com_pos[0][2]};
                array<float, 3> current_order_left =  {com_pos[1][0], com_pos[1][1], com_pos[1][2]};
                float current_theta_right = com_pos[0][3];
                float current_theta_left =  com_pos[1][3];

                array<float, 3> diff_right = {0.0f - current_order_right[0],  0.04f - current_order_right[1], 0.08f - current_order_right[2]};
                array<float, 3> diff_left =  {0.0f - current_order_left[0] , -0.04f - current_order_left[1] , 0.08f - current_order_left[2]};
                float diff_theta_right = 0.0f - current_theta_right;
                float diff_theta_left = 0.0f - current_theta_left;

                int step = int(0.3 * CTRL_STEP);
                for(int i=0; i<=step; i++){
                    array<float, 3> motion_right;
                    array<float, 3> motion_left;

                    for(int id = 0; id<3; id++){
                        motion_right[id] = current_order_right[id] + diff_right[id]*( (float)(i) ) / (step);
                        motion_left[id]  = current_order_left[id]  + diff_left[id] *( (float)(i) ) / (step);
                    }

                    robot->move_leg_ik(motion_right, current_theta_right + diff_theta_right * ( (float)(i) ) / (step), 0.0, true);
                    robot->move_leg_ik(motion_left,  current_theta_left  + diff_theta_left  * ( (float)(i) ) / (step), 0.0, false);
        
                    delay(CTRL_CYCLE);
                }
                delay(600);

                // phase transition
                init_phase(
                    Mode::WALK,
                    Phase::WAKE,
                    0
                );
                break;
            }

            case Phase::WAKE:{
                // robot->move_link_t(1, 3.14/2.5, 0.8);
                // robot->move_link_t(4, 3.14/2.5, 0.8);
                if (sensor.face_up()){
                    wake_face_up();
                }else{
                    wake_face_down();
                }
                order_free = false;

                // phase transition
                init_phase(
                    Mode::WAIT,
                    Phase::START,
                    0
                );
                break;
            }
        }

        /* #########################################################################
        ##########################################################################*/
        // sensor feedback
        sensor.update();
        array<float, 2> ideal_acc = {com_pos[2][0], com_pos[2][1]};
        array<float, 2> angle_com_pos_fb = sensor.angle_com_pos_fb();
        float delay_duration = sensor.delay_duration_fb(ideal_acc, CTRL_STEP);

        array<float, 2> com_pos_fb = {
            angle_com_pos_fb[0],
            angle_com_pos_fb[1]
        };

        // dummy
        // float delay_duration = 1000.0f / CTRL_STEP;
        // array<float, 2> com_pos_fb = {0,0};

        // arm feedback
        array<float, 3> arm_pos_right = robot->arm_k_solver({global_control_pkt.arm_right[0], global_control_pkt.arm_right[1], global_control_pkt.arm_right[2]});
        array<float, 3> arm_pos_left  = robot->arm_k_solver({global_control_pkt.arm_left[0], global_control_pkt.arm_left[1], global_control_pkt.arm_left[2]});
        array<float, 2> arm_mass_pos = {arm_pos_right[0] + arm_pos_left[0], arm_pos_right[1] + arm_pos_left[1]};

        array<float, 2> com_diff = {arm_mass_pos[0] / 6, arm_mass_pos[1] / 6};
        
        // move robot
        // devide com pos into each leg
        array<float, 3> leg_right_com = {
            com_pos[0][0] - com_pos_fb[0] - com_diff[0] * cos(com_pos[0][3]) - com_diff[1] * sin(com_pos[0][3]), 
            com_pos[0][1] - com_pos_fb[1] - com_diff[0] * sin(com_pos[0][3]) - com_diff[1] * cos(com_pos[0][3]), 
            com_pos[0][2]};
        array<float, 3> leg_left_com =  {
            com_pos[1][0] - com_pos_fb[0] - com_diff[1] * cos(com_pos[1][3]) - com_diff[0] * sin(com_pos[1][3]), 
            com_pos[1][1] - com_pos_fb[1] - com_diff[1] * sin(com_pos[1][3]) - com_diff[0] * cos(com_pos[1][3]),
            com_pos[1][2]};
        // array<float, 3> leg_right_com = {
        //     com_pos[0][0] - com_pos_fb[0], 
        //     com_pos[0][1] - com_pos_fb[1], 
        //     com_pos[0][2]};
        // array<float, 3> leg_left_com =  {
        //     com_pos[1][0] - com_pos_fb[0], 
        //     com_pos[1][1] - com_pos_fb[1],
        //     com_pos[1][2]};

        // send order
        robot->move_leg_ik(leg_right_com, com_pos[0][3], 0.0, true);
        robot->move_leg_ik(leg_left_com, com_pos[1][3], 0.0, false);

        // robot->move_leg_ik({- com_pos_fb[0], 0.04 - com_pos_fb[1], 0.158}, com_pos[0][3], 0.0, true);
        // robot->move_leg_ik({- com_pos_fb[0], -0.04 - com_pos_fb[1], 0.158}, com_pos[1][3], 0.0, false);

        /* #########################################################################
        ##########################################################################*/
        // delay
        vTaskDelay(pdMS_TO_TICKS(delay_duration));
    }
}

void wake_face_up(){
    Serial.println("Wake up face up");

    array<float, LINK_SIZE> motion;

    motion = sd->read_motion("/wake_face_up.txt", 0);
    robot->move_all_t(motion, 0.5);

    motion = sd->read_motion("/wake_face_up.txt", 1);
    robot->move_all_t(motion, 0.5);

    motion = sd->read_motion("/wake_face_up.txt", 2);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_up.txt", 3);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_up.txt", 4);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_up.txt", 5);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_up.txt", 6);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_up.txt", 7);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_up.txt", 8);
    robot->move_all_t(motion, 0.5);

    robot->init_home(1.5);
}

void wake_face_down(){
    Serial.println("Wake up face down");

    array<float, LINK_SIZE> motion;

    motion = sd->read_motion("/wake_face_down.txt", 0);
    robot->move_all_t(motion, 0.5);

    motion = sd->read_motion("/wake_face_down.txt", 1);
    robot->move_all_t(motion, 0.5);

    motion = sd->read_motion("/wake_face_down.txt", 2);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_down.txt", 3);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_down.txt", 4);
    robot->move_all_t(motion, 1.5);

    motion = sd->read_motion("/wake_face_down.txt", 5);
    robot->move_all_t(motion, 0.5);

    robot->init_home(1.5);
}
