#include "lower_body.h"

static array<float, 3> vd = {0.0f, 0.0f, 0.0f};

// orders
static Order order;

// stances
STANCE_INFO stance_walk = {
    .height_diff = 0.0f,
    .relative_body_angle = 0.0f,
    .relative_body_pos = 0.0f,
    .relative_leg_angle = 0.0f
};
STANCE_INFO stance_fight = {
    .height_diff = 0.0f,
    .relative_body_angle = 30.0 * PI / 180.0f,
    .relative_body_pos = 0.0f,
    .relative_leg_angle = -60.0 * PI / 180.0f
};
STANCE_INFO stance = stance_walk;
STANCE_INFO stance_diff = stance_walk;

// mode and phase
static Mode mode = Mode::WAIT;
static Mode mode_last = Mode::WALK;

static Phase phase = Phase::WAIT;
static Phase phase_next;
static Phase phase_last = Phase::WAIT;

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

    sensor.init();
    delay(1000);
    sensor.update();
    delay(1000);
    Serial.println("Lower body control initialized");

    // initialize control parameters
    controller.init_param_walk(HEIGHT_WALK);
    controller.init_pose();
}

array<float, 3> update_vel(array<float, 3> vd, Order order){
    // if (order == Order::CROUCH || order == Order::UNCROUCH){
    //     // If crouching or uncrouching, set velocity to zero
    //     vd[0] = 0.0f;
    //     vd[1] = 0.0f;
    //     vd[2] = 0.0f;
    //     return vd;
    // }

    // Serial.print("vd: "); Serial.print(vd[0], 4); Serial.print(", "); Serial.print(vd[1], 4); Serial.print(", "); Serial.println(vd[2], 4);
    array<float, 3> vd_max_abs = controller.get_vd_max_abs();
    vd[0] = global_control_pkt.stick_right[0] * vd_max_abs[0];
    vd[1] = global_control_pkt.stick_right[1] * vd_max_abs[1];
    vd[2] = global_control_pkt.stick_left[1] * vd_max_abs[2];
    return vd;
}

void init_phase(Mode next_mode, Phase next_phase, float next_phase_time){
    mode_last = mode;
    mode = next_mode;

    phase_last = phase;
    phase = next_phase;

    phase_length = next_phase_time * CTRL_STEP;
    phase_count = 0;
}

void update_phase(){
    // stop if no movement
    if (controller.p_n2p1_equels_p_n2m1() && 
        abs(vd[0]) < VD_MIN && abs(vd[1]) < VD_MIN && abs(vd[2]) < VD_MIN){
        phase_next = Phase::END;
    }
    else{
        phase_next = Phase::DOUBLE;
    }
}

array<array<float, 5>, 3> attach_stance(array<array<float, 5>, 3> com_pos, STANCE_INFO stance){
    // // height
    com_pos[0][2] += stance.height_diff;
    com_pos[1][2] += stance.height_diff;

    // relative body angle
    com_pos[0][3] += stance.relative_body_angle;
    com_pos[1][3] += stance.relative_body_angle;

    // left leg angle
    com_pos[1][3] += stance.relative_leg_angle;

    // adjust rotation to com pos
    float px_r = com_pos[0][0]; float py_r = com_pos[0][1];
    float theta_r = com_pos[0][3];
    float pl_x = com_pos[1][0]; float py_l = com_pos[1][1];
    float theta_l = com_pos[1][3];

    com_pos[0][0] =  px_r * cos(theta_r) + py_r * sin(theta_r);
    com_pos[0][1] = -px_r * sin(theta_r) + py_r * cos(theta_r);

    com_pos[1][0] =  pl_x * cos(theta_l) + py_l * sin(theta_l);
    com_pos[1][1] = -pl_x * sin(theta_l) + py_l * cos(theta_l);

    return com_pos;
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
    Serial.println("Core1Task started");

    while(1) {
        sensor.update();
        /* #########################################################################
        LED HANDLER */
        array<int,3> RED    = {255, 0,   0  }; // : disconnected or free
        array<int,3> WHITE  = {255, 255, 255}; // : WAIT
        array<int,3> BLUE   = {0,   0,   255}; // : WALK
        array<int,3> GREEN  = {0,   255, 0  }; // : CROUCH
        array<int,3> YELLOW = {255, 255, 0  }; // : FIGHT
        /* ###################################################################### */
        if(!connected || mode == Mode::FREE){
            neopixelWrite(RGB_BUILTIN, RED[0], RED[1], RED[2]);
        }else if (mode == Mode::WAIT){
            neopixelWrite(RGB_BUILTIN, WHITE[0], WHITE[1], WHITE[2]);
        }else if (mode == Mode::WALK){
            neopixelWrite(RGB_BUILTIN, BLUE[0], BLUE[1], BLUE[2]);
        }else if (order == Order::CROUCH){
            neopixelWrite(RGB_BUILTIN, GREEN[0], GREEN[1], GREEN[2]);
        }else if (mode == Mode::FIGHT){
            neopixelWrite(RGB_BUILTIN, YELLOW[0], YELLOW[1], YELLOW[2]);
        }else{
            neopixelWrite(RGB_BUILTIN, RED[0], RED[1], RED[2]);
        }
        /* #########################################################################
        CONTROLLER HANDLER
        handle controller order. 
        dont change order while first order is not executed.
        Basic Orders: 
        - MODE_CHANGE : Change mode between WALK and FIGHT. Change stance and control parametres whiile last half of SINGLE phase (which labeled as STANCE).
        - ROTATE      : dynamically change the body angle. Define as the function of STANCE.

        Orders while WALK mode:
        CROUCH        : Crouch the robot. transit to END before CROUCH. 
        UNCROUCH      : Return to WALK mode.
        ######################################################################### */
        if(order == Order::NONE){
            // MODE_CHENGE
            if (global_control_pkt.button_right[0] == 0){
                order = Order::MODE_CHANGE;
            }
            else if (mode == Mode::WALK){
                // CROUCH
                if (global_control_pkt.button_left[2] == 0){
                    order = Order::CROUCH;
                }
            }
            else if (mode == Mode::FIGHT){
            }
        }

        // UNCROUCH
        if (order == Order::CROUCH){
            if (global_control_pkt.button_left[2] == 1){
                order = Order::UNCROUCH;
            }
        }
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
            init_phase(
                Mode::FREE,
                Phase::WAIT,
                0
            );
            robot->free_all();
            continue;
        }
        // if last order is "order_free", start from falling phase
        if(order_free){
            init_phase(
                Mode::WALK,
                Phase::WAKE,
                0
            );
        }
        // fall check
        if(sensor.fall() && phase != Phase::FALL && phase != Phase::WAKE){
            init_phase(
                Mode::WALK,
                Phase::FALL,
                0
            );
        }

        // update vd
        vd = update_vel(vd, order);
        array<float, 3> vd_fb = sensor.vd_fb(vd);
        // vd[0] += vd_fb[0];
        // vd[1] += vd_fb[1];
        // vd[2] += vd_fb[2];

        // walk if vd is large enough or in MODE_CHANGE order
        if (mode == Mode::WAIT){
            if (abs(vd[0]) > VD_MIN || abs(vd[1]) > VD_MIN || abs(vd[2]) > VD_MIN || order == Order::MODE_CHANGE){
                init_phase(
                    mode_last,
                    Phase::START,
                    0
                );
            }
        }

        /* #########################################################################
        TRAJECTORY CALCULATION AND PHASE UPDATE
        In the second step, calculate the desired com position based on the phase. Update the phase at the end of each phase duration
        For each phase,
        - STANCE   : Start from middle point of SINGLE. decide next foot position. Next phase is DOUBLE.
        - START    : Initialize satrt parameters and phase length. Next phase is SINGLE.
        - END      : Initialize end parameters and phase length. Next phase is START, and change mode to WAIT. CROUCH / UNCROUCH before phase initialization when order given.
        - SINGLE   : At the middle of the phase, decide next phase and next foot position. Next phase is (DOUBLE / END / ).
        - DOUBLE   : CoM transition between SINGLE and SINGLE. calculate next SINGLE phase parameters and change pivot in the first step. Next phase is SINGLE.
        - FLIGHT   : 
        - FALL     : After slip is detected, free upper body and shrink lower body for the safety. Next phase is WAKE.
        - WAKE     : WAKE the robot up. Next phase is START. and change the mode to WAIT.
        - WAIT     : Do nothing.
        ##########################################################################*/
        // move robot
        // init com_pos
        array<array<float, 5>, 3> com_pos = controller.get_default_com_pos();
        // phase switch-case sentences
        switch (phase){
            case Phase::STANCE:{
                if (phase_count == 0){
                    Serial.println("phase: STANCE");
                    if (order == Order::MODE_CHANGE){
                        // change calculation parameters except height
                        if (mode_last == Mode::WALK){
                            controller.init_param_fight(HEIGHT_FIGHT);
                            stance.height_diff = HEIGHT_WALK - HEIGHT_FIGHT;
                            stance_diff.height_diff = (HEIGHT_FIGHT - HEIGHT_WALK) / phase_length;
                            stance_diff.relative_body_angle = 
                                (stance_fight.relative_body_angle - stance.relative_body_angle) / phase_length;
                            stance_diff.relative_body_pos = 
                                (stance_fight.relative_body_pos - stance.relative_body_pos) / phase_length;
                            stance_diff.relative_leg_angle = 
                                (stance_fight.relative_leg_angle - stance.relative_leg_angle) / phase_length;
                            // change mode from TRANSITION to FIGHT
                            mode = Mode::FIGHT;
                            mode_last = Mode::WALK;
                        }else if (mode_last == Mode::FIGHT){
                            controller.init_param_walk(HEIGHT_WALK);
                            stance.height_diff = HEIGHT_FIGHT - HEIGHT_WALK;
                            stance_diff.height_diff = (HEIGHT_WALK - HEIGHT_FIGHT) / phase_length;
                            stance_diff.relative_body_angle = 
                                (stance_walk.relative_body_angle - stance.relative_body_angle) / phase_length;
                            stance_diff.relative_body_pos = 
                                (stance_walk.relative_body_pos - stance.relative_body_pos) / phase_length;
                            stance_diff.relative_leg_angle = 
                                (stance_walk.relative_leg_angle - stance.relative_leg_angle) / phase_length;
                            // change mode from TRANSITION to WALK
                            mode = Mode::WALK;
                            mode_last = Mode::FIGHT;
                        }

                        controller.update_state_variables({0,0,0}, {0,0}, 0);
                        controller.init_single_half();
                        order = Order::NONE;

                    }else if(order == Order::ROTATE){
                        float body_angle_order;
                        if (global_control_pkt.stick_left[0] < 0){
                            body_angle_order = -0.1f;
                        }
                        controller.update_state_variables({0,0,0}, {0,0}, body_angle_order);
                        controller.init_single_half();
                        order = Order::NONE;
                    }
                }

                com_pos = controller.calc_com_traj_single(phase_count / (float)CTRL_STEP);
                stance.height_diff += stance_diff.height_diff;
                stance.relative_body_angle += stance_diff.relative_body_angle;
                stance.relative_body_pos += stance_diff.relative_body_pos;
                stance.relative_leg_angle += stance_diff.relative_leg_angle;

                // phase transition
                phase_count++;
                if (phase_count == phase_length){
                    stance_diff = stance_walk;
                    init_phase(
                        mode,
                        Phase::DOUBLE, 
                        controller.get_T_ds()
                    );
                }
                break;
            }

            case Phase::START:{
                if (phase_count == 0){
                    Serial.println("phase: START");
                    if (order == Order::CROUCH){
                        controller.init_param_walk(HEIGHT_CROUCH);
                    }else if (mode == Mode::FIGHT){
                        controller.init_param_fight(HEIGHT_FIGHT);
                    }else{
                        controller.init_param_walk(HEIGHT_WALK);
                    }
                    controller.init_pose();
                    controller.inverse_pivot();
                    controller.init_start();
                    float T_ds = controller.get_T_sup() * controller.get_ds_ratio() * 0.5f;
                    phase_length = T_ds * CTRL_STEP;
                }
                com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP);
                
                // phase transition
                phase_count++;
                if (phase_count == phase_length){
                    init_phase(
                        mode,
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
                    // phase_length = T_ds * CTRL_STEP;
                    phase_length = 1;
                }
                // com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP);
                // Serial.print("com_pos: "); Serial.print(com_pos[0][0], 4); Serial.print(", "); Serial.print(com_pos[0][1], 4); Serial.print(", "); Serial.println(com_pos[0][2], 4);
                com_pos = controller.get_default_com_pos();

                // phase transition
                phase_count++;
                if (phase_count == phase_length){
                    // CROUCH or UNCROUCH according to order
                    if (order == Order::CROUCH || order == Order::UNCROUCH){
                        crouch(order, com_pos);
                    }
                    init_phase(
                        Mode::WAIT,
                        Phase::WAIT,
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
                    // change phase to STANCE if order is mode_change
                    if (order == Order::MODE_CHANGE && controller.pivot_right()){
                        init_phase(
                            Mode::TRANSITION,
                            Phase::STANCE,
                            controller.get_T_sup()*(1.0f - controller.get_ds_ratio()) / 2
                        );
                        break;
                    }else if (order == Order::ROTATE){
                        init_phase(
                            mode,
                            Phase::STANCE,
                            controller.get_T_sup()*(1.0f - controller.get_ds_ratio()) / 2
                        );
                        break;
                    }

                    // sensor feedback
                    // array<float, 2> foot_pos_fb = sensor.foot_pos_fb();
                    // sensor.init_norm();
                    array<float, 2> foot_pos_fb = {0,0};
                    // update state variables in gait controller
                    controller.update_state_variables(vd, foot_pos_fb, 0);
                    controller.init_single_half();
                    update_phase();
                }
                com_pos = controller.calc_com_traj_single(phase_count / (float)CTRL_STEP);

                // phase transition
                phase_count++;
                if (phase_count == phase_length){
                    init_phase(
                        mode,
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
                        mode,
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
                Serial.println("phase: FALL");
                order_free = true;
                robot->free_upper();
                array<float, 3> current_order_right = {com_pos[0][0], com_pos[0][1], com_pos[0][2]};
                array<float, 3> current_order_left =  {com_pos[1][0], com_pos[1][1], com_pos[1][2]};
                float current_theta_right = com_pos[0][3];
                float current_theta_left =  com_pos[1][3];

                robot->move_safely_fall(
                    current_order_right,
                    current_theta_right,
                    current_order_left,
                    current_theta_left,
                    0.06f,
                    0.2f
                );
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
                Serial.println("phase: WAKE");
                if (sensor.face_up()){
                    wake_face_up();
                }else{
                    wake_face_down();
                }
                order_free = false;

                // phase transition
                init_phase(
                    Mode::WAIT,
                    Phase::WAIT,
                    0
                );
                break;
            }

            case Phase::WAIT:{
                break;
            }
        }

        /* #########################################################################
        FEEDBACK
        - attach stance
        - sensor feedback
        - motion feedback
        ##########################################################################*/
        float delay_duration = 1000.0f / CTRL_STEP;
        // attach stance
        com_pos = attach_stance(com_pos, stance);
        // in START phase, dont make swing leg
        if (phase == Phase::START || phase_last == Phase::START){
            float height = max(com_pos[0][2], com_pos[1][2]);
            com_pos[0][2] = height;
            com_pos[1][2] = height;
            delay_duration *= 2;
        }
        // sensor feedback
        array<float, 2> angle_com_pos_fb = sensor.angle_com_pos_fb();
        array<float, 2> com_pos_fb = {
            angle_com_pos_fb[0],
            angle_com_pos_fb[1]
        };

        if (phase == Phase::SINGLE){
            array<float, 2> ideal_acc = {com_pos[2][0], com_pos[2][1]};
            float Tc = controller.get_Tc();
            float t_ideal = phase_count / (float)CTRL_STEP + controller.get_T_ds()/2;
            delay_duration = sensor.delay_duration_fb(controller.get_approx_coeff(), ideal_acc, Tc, t_ideal, CTRL_STEP);
        }

        // dummy feedback
        // com_pos_fb = {0,0};
        // float delay_duration = 1000.0f / CTRL_STEP;

        // arm feedback
        array<float, 3> arm_pos_right = robot->arm_k_solver({global_control_pkt.arm_right[0], global_control_pkt.arm_right[1], global_control_pkt.arm_right[2]});
        array<float, 3> arm_pos_left  = robot->arm_k_solver({global_control_pkt.arm_left[0], global_control_pkt.arm_left[1], global_control_pkt.arm_left[2]});
        array<float, 2> arm_mass_pos = {arm_pos_right[0] + arm_pos_left[0], -arm_pos_right[1] + arm_pos_left[1]};
        array<float, 2> com_diff = {arm_mass_pos[0] / 12 , arm_mass_pos[1] / 12};

        com_pos_fb[0] += com_diff[0];
        com_pos_fb[1] += com_diff[1];
        
        // move robot
        // devide com pos into each leg
        array<float, 3> leg_right_com = {
            com_pos[0][0] - com_pos_fb[0],
            com_pos[0][1] - com_pos_fb[1],
            com_pos[0][2]};
        array<float, 3> leg_left_com =  {
            com_pos[1][0] - com_pos_fb[0],
            com_pos[1][1] - com_pos_fb[1],
            com_pos[1][2]};

        // Serial.println("com_pos:");
        // Serial.print(com_pos[0][0]); Serial.print(", "); Serial.print(com_pos[0][1]); Serial.print(", "); Serial.println(com_pos[0][2]);

        // send order
        if (phase == Phase::FALL || phase == Phase::WAKE){
            // do nothing
            continue;
        }
        float phi_fb = sensor.angle_phi_fb();
        robot->move_leg_ik(leg_right_com, com_pos[0][3], phi_fb, true);
        robot->move_leg_ik(leg_left_com, com_pos[1][3] , phi_fb, false);
        com_x[0] = leg_right_com[0];
        com_x[1] = leg_left_com[0];

        /* #########################################################################
        DELAY for NEXT CYCLE
        ##########################################################################*/
        // delay
        vTaskDelay(pdMS_TO_TICKS(delay_duration));
        // vTaskDelay(pdMS_TO_TICKS(1000.0f / CTRL_STEP));
    }
}

void crouch(Order order, array<array<float, 5>, 3>& com_pos){
    // similar movement as FALL
    array<float, 3> current_order_right = {com_pos[0][0], com_pos[0][1], com_pos[0][2]};
    array<float, 3> current_order_left =  {com_pos[1][0], com_pos[1][1], com_pos[1][2]};
    float current_theta_right = com_pos[0][3];
    float current_theta_left =  com_pos[1][3];

    float height;
    if (order == Order::CROUCH){
        Serial.println("Crouch");
        height = HEIGHT_CROUCH;
    }else if (order == Order::UNCROUCH){
        Serial.println("Uncrouch");
        height = HEIGHT_WALK;
        mode = Mode::WALK;
        order = Order::NONE;
    }

    robot->move_safely_fall(
        current_order_right,
        current_theta_right,
        current_order_left,
        current_theta_left,
        height,
        0.8f
    );

}

void wake_face_up(){
    Serial.println("Wake face up");

    sd->play_motion(robot, "/wake_face_up.csv", 0.35f);
    robot->init_home(1);
}

void wake_face_down(){
    Serial.println("Wake face down");

    sd->play_motion(robot, "/wake_face_down.csv", 0.35f);
    robot->init_home(1);
}
