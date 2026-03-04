#include "lower_body.h"

// orders
static array<float, 3> vd = {0.0f, 0.0f, 0.0f};
static Order order;

// stances
STANCE_INFO stance_walk = {
    .height_diff         = 0.0f,
    .relative_body_angle = 0.0f,
    .relative_body_pos   = 0.0f,
    .relative_leg_angle  = 0.0f
};
STANCE_INFO stance_fight = {
    .height_diff         = 0.0f,
    .relative_body_angle = 30.0 * PI / 180.0f,
    .relative_body_pos   = 0.0f,
    .relative_leg_angle  = -60.0 * PI / 180.0f
};
STANCE_INFO stance_crouch = stance_walk;
STANCE_INFO stance = stance_walk;
STANCE_INFO stance_next;
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
static int phase_count_x;
static int update_rate = UPDATE_RATE_BASE;
bool single_calculated = true;

// body angle
float body_angle_order = 0.0f;
float body_angle = 0.0f;
float body_angle_last = body_angle;
float kp_body_angle = 0.5f;
float kd_body_angle = 0.01f;

// control classes
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
    if (!connected){
        vd = {0.0f, 0.0f, 0.0f};
        return vd;
    }
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

    phase_length = next_phase_time * CTRL_STEP * UPDATE_RATE_BASE;

    phase_count_x = 0;
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

    // rotate
    array<float, 2> p_r = controller.rotate_vec(
        {com_pos[0][0], com_pos[0][1]},
        stance.relative_body_angle
    );
    array<float, 2> p_l = controller.rotate_vec(
        {com_pos[1][0], com_pos[1][1]},
        stance.relative_body_angle + stance.relative_leg_angle
    );
    com_pos[0][0] =  p_r[0];
    com_pos[0][1] =  p_r[1];

    com_pos[1][0] =  p_l[0];
    com_pos[1][1] =  p_l[1];

    return com_pos;
}

STANCE_INFO update_stance_diff(
    STANCE_INFO stance_next, int phase_length,
    float height_aim, float height_now,
    Mode mode_next)
{
    STANCE_INFO stance_diff_updated;

    if (mode_next == Mode::FIGHT){
        controller.init_param_fight(height_aim);
    }else{
        controller.init_param_walk(height_aim);
    }

    stance_diff_updated.height_diff = (height_aim - height_now) / phase_length * UPDATE_RATE_BASE;
    stance_diff_updated.relative_body_angle = 
        (stance_next.relative_body_angle - stance.relative_body_angle) / phase_length * UPDATE_RATE_BASE;
    stance_diff_updated.relative_body_pos = 
        (stance_next.relative_body_pos - stance.relative_body_pos) / phase_length * UPDATE_RATE_BASE;
    stance_diff_updated.relative_leg_angle = 
        (stance_next.relative_leg_angle - stance.relative_leg_angle) / phase_length * UPDATE_RATE_BASE;

    return stance_diff_updated;
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
        }else if (mode == Mode::CROUCH){
            neopixelWrite(RGB_BUILTIN, GREEN[0], GREEN[1], GREEN[2]);
        }else if (mode == Mode::FIGHT){
            neopixelWrite(RGB_BUILTIN, YELLOW[0], YELLOW[1], YELLOW[2]);
        }else{
            neopixelWrite(RGB_BUILTIN, RED[0], RED[1], RED[2]);
        }

        /* #########################################################################
        CONTROLLER HANDLER
        handle controller order. 
        Mode/Phase change order will be executed after first order is finished.
        - Motion Control orders:
          - body angle order: change body angle accordance with button[1]. move larger if FIGHT mode.
          - 

        - Mode/Phase chaange orders
          - Basic Orders: 
            - MODE_CHANGE : Change mode between WALK and FIGHT. Change stance and control parametres whiile last half of SINGLE phase (which labeled as STANCE).
 
          - Orders while WALK mode:
            - CROUCH      : Crouch the robot. STANCE. 

          - Orders while CROUCH mode:
            - STAND       : Uncrouch the robot. STANCE.

          - Orders while FIGHT mode:
        ######################################################################### */
        // Motion control orders
        // body angle order
        if (global_control_pkt.button_right[1] == 0){
            if (mode == Mode::FIGHT){
                body_angle_order = BODY_ANGLE_LARGE;
            }else{
                body_angle_order = BODY_ANGLE_SMALL;
            }
        }else if (global_control_pkt.button_left[1] == 0){
            body_angle_order = -BODY_ANGLE_SMALL;
        }else{
            body_angle_order = 0.0f;
        }

        // Mode/Phase change orders
        if(order == Order::NONE){
            // basic orders
            if (global_control_pkt.button_right[0] == 0){
                order = Order::MODE_CHANGE;
            }
            // orders while WALK mode
            else if (mode == Mode::WALK){
                // CROUCH
                if (global_control_pkt.button_left[0] == 0){
                    order = Order::CROUCH;
                }
            }
            // orders while CROUCH mode
            else if (mode == Mode::CROUCH){
                // CROUCH
                if (global_control_pkt.button_left[0] == 0){
                    order = Order::STAND;
                }
            }
            // orders while FIGHT mode
            else if (mode == Mode::FIGHT){
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
        // init com_pos
        array<array<float, 5>, 3> com_pos = controller.get_default_com_pos();
        // save t_ideal before phase_count updated (for update_rate feedback)
        float t_ideal = phase_count   / (float)CTRL_STEP / (float)UPDATE_RATE_BASE + controller.get_T_ds()/2;
        float tx      = phase_count_x / (float)CTRL_STEP / (float)UPDATE_RATE_BASE + controller.get_T_ds()/2;
        // phase switch-case sentences
        switch (phase){
            case Phase::STANCE:{
                if (phase_count == 0){
                    Serial.println("phase: STANCE");
                    float height_now;
                    float height_aim;

                    // set height, stance, and mode
                    if (order == Order::MODE_CHANGE){
                        if (mode_last == Mode::WALK){
                            height_now = HEIGHT_WALK; height_aim = HEIGHT_FIGHT;
                            stance_next = stance_fight;
                            mode = Mode::FIGHT; mode_last = Mode::WALK;
                        }else if (mode_last == Mode::FIGHT){
                            height_now = HEIGHT_FIGHT; height_aim = HEIGHT_WALK;
                            stance_next = stance_walk;
                            mode = Mode::WALK; mode_last = Mode::FIGHT;
                        }

                    }else if (order == Order::CROUCH){
                        height_now = HEIGHT_WALK; height_aim = HEIGHT_CROUCH;
                        stance_next = stance_crouch;
                        mode = Mode::CROUCH; mode_last = Mode::WALK;

                    }else if (order == Order::STAND){
                        height_now = HEIGHT_CROUCH; height_aim = HEIGHT_WALK;
                        stance_next = stance_walk;
                        mode = Mode::WALK; mode_last = Mode::CROUCH;
                    }

                    // preparation for stance update
                    stance.height_diff = height_now - height_aim;
                    stance_diff = update_stance_diff(
                        stance_next, phase_length,
                        height_aim, height_now,
                        mode);
                    controller.update_state_variables({0,0,0});
                }

                com_pos = controller.calc_com_traj_single(
                    phase_count_x / (float)CTRL_STEP / (float)UPDATE_RATE_BASE,
                    phase_count   / (float)CTRL_STEP / (float)UPDATE_RATE_BASE
                );
                stance.height_diff         += stance_diff.height_diff;
                stance.relative_body_angle += stance_diff.relative_body_angle;
                stance.relative_body_pos   += stance_diff.relative_body_pos;
                stance.relative_leg_angle  += stance_diff.relative_leg_angle;

                // phase transition
                phase_count_x += UPDATE_RATE_BASE;
                phase_count   += update_rate;
                if (phase_count >= phase_length){
                    stance = stance_next;
                    order = Order::NONE;
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
                    if (mode == Mode::CROUCH){
                        controller.init_param_walk(HEIGHT_CROUCH);
                        stance = stance_walk;
                    }else if (mode == Mode::FIGHT){
                        controller.init_param_fight(HEIGHT_FIGHT);
                        stance = stance_fight;
                    }else{
                        controller.init_param_walk(HEIGHT_WALK);
                        stance = stance_walk;
                    }
                    controller.init_pose();
                    controller.inverse_pivot();
                    controller.init_state_variables(true, false);
                    phase_length = controller.get_T_ds() * CTRL_STEP * UPDATE_RATE_BASE;
                }
                com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP / (float)UPDATE_RATE_BASE);
                
                // phase transition
                phase_count += update_rate;
                if (phase_count >= phase_length){
                    init_phase(
                        mode,
                        Phase::SINGLE, 
                        controller.get_T_sup() - controller.get_T_ds()
                    );
                }
                break;
            }
            
            case Phase::END:{
                if (phase_count == 0){
                    Serial.println("phase: END");
                    controller.inverse_pivot();
                    controller.init_state_variables(false, true);
                    float T_ds = controller.get_T_ds()/2;
                    // phase_length = T_ds * CTRL_STEP * UPDATE_RATE_BASE;
                    phase_length = 1;
                }
                // com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP / (float)UPDATE_RATE_BASE);
                // Serial.print("com_pos: "); Serial.print(com_pos[0][0], 4); Serial.print(", "); Serial.print(com_pos[0][1], 4); Serial.print(", "); Serial.println(com_pos[0][2], 4);
                com_pos = controller.get_default_com_pos();

                // phase transition
                phase_count += update_rate;
                if (phase_count >= phase_length){
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
                    controller.init_single();
                    single_calculated = false;
                }
                if (!single_calculated && phase_count >= int(phase_length/2)){
                    single_calculated = true;
                    // change phase to STANCE if order is given
                    if ((order == Order::MODE_CHANGE || order == Order::CROUCH || order == Order::STAND)
                        && controller.pivot_right())
                    {
                        init_phase(
                            Mode::TRANSITION,
                            Phase::STANCE,
                            (controller.get_T_sup() - controller.get_T_ds()) / 2
                        );
                        break;
                    }

                    // update state variables in gait controller
                    controller.update_state_variables(vd);
                    update_phase();
                }
                com_pos = controller.calc_com_traj_single(
                    phase_count_x / (float)CTRL_STEP / (float)UPDATE_RATE_BASE,
                    phase_count   / (float)CTRL_STEP / (float)UPDATE_RATE_BASE
                );

                // phase transition
                phase_count_x += UPDATE_RATE_BASE;
                phase_count   += update_rate;
                if (phase_count >= phase_length){
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
                    controller.init_state_variables(false, false);
                }
                com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP / (float)UPDATE_RATE_BASE);
                
                // phase transition
                phase_count += update_rate;
                if (phase_count >= phase_length){
                    init_phase(
                        mode,
                        Phase::SINGLE, 
                        controller.get_T_sup() - controller.get_T_ds()
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
                robot->free_all();
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
        - sensor feedback (angle)
        - sensor feedback (update rate)
        - sensor feedback (x0 and vx0)
        - START exception
        - arm position feedback
        ##########################################################################*/
        if (phase == Phase::FALL || phase == Phase::WAKE){
            // do nothing
            continue;
        }
        // attach stance
        com_pos = attach_stance(com_pos, stance);

        // sensor feedback
        // angle feedback
        array<float, 2> angle_com_pos_fb = sensor.angle_com_pos_fb();
        array<float, 2> com_pos_fb = {
            angle_com_pos_fb[0],
            angle_com_pos_fb[1]
        };

        // update_rate feedback
        array<float, 2> acc_ideal = {com_pos[2][0], com_pos[2][1]};
        float Tc = controller.get_Tc();
        if (phase == Phase::SINGLE){
            // com calculation check
            float com_y_pos;
            if(controller.is_pivot_right()){
                com_y_pos = com_pos[0][1];
            }else{
                com_y_pos = com_pos[1][1];
            }

            update_rate = sensor.update_rate_fb(
                t_ideal, acc_ideal,
                controller.get_approx_coeff_y(), Tc, UPDATE_RATE_BASE,
                com_y_pos
            );
        }else{
            update_rate = UPDATE_RATE_BASE;
        }
        controller.update_T_sup_x(1/CTRL_STEP * (UPDATE_RATE_BASE - update_rate)/UPDATE_RATE_BASE);

        // at START and phase one after, dont make swing leg, and move slowly
        if (phase == Phase::START || phase_last == Phase::START){
            float height = max(com_pos[0][2], com_pos[1][2]);
            com_pos[0][2] = height;
            com_pos[1][2] = height;
            update_rate = (int)(UPDATE_RATE_BASE / 2);
        }

        // x0 and vx0 feedback
        if (phase == Phase::SINGLE){
            array<float, 2> x0_vx0    = controller.get_x0_vx0();
            array<float, 2> x0_vx0_fb = sensor.x0_vx0_fb(
                tx,
                x0_vx0[0], x0_vx0[1],
                Tc, CTRL_STEP
            );
            controller.feedback_x0_vx0(x0_vx0_fb);
        }

        // arm position feedback
        array<float, 3> arm_right_pos = robot->arm_k_solver({arm_right_angles[0], arm_right_angles[1], arm_right_angles[2]});
        array<float, 3> arm_left_pos  = robot->arm_k_solver({arm_left_angles[0], arm_left_angles[1], arm_left_angles[2]});
        array<float, 2> arm_mass_pos  = {arm_right_pos[0] + arm_left_pos[0], -arm_right_pos[1] + arm_left_pos[1]};
        array<float, 2> com_diff      = {arm_mass_pos[0] / 12 , arm_mass_pos[1] / 12};

        com_pos_fb[0] += com_diff[0];
        com_pos_fb[1] += com_diff[1];

        // dummy feedback
        // com_pos_fb = {0,0};
        // update_rate = UPDATE_RATE_BASE;
        
        /* #########################################################################
        EXECUTION
        - move robot
        - delay
        ##########################################################################*/
        // move robot
        // devide com pos into each leg
        array<float, 2> fb_r = controller.rotate_vec(
            {com_pos_fb[0], com_pos_fb[1]}, com_pos[0][3]);
        array<float, 2> fb_l = controller.rotate_vec(
            {com_pos_fb[0], com_pos_fb[1]}, com_pos[1][3]);
        array<float, 3> leg_right_com = {
            com_pos[0][0] - fb_r[0],
            com_pos[0][1] - fb_r[1],
            com_pos[0][2]};
        array<float, 3> leg_left_com =  {
            com_pos[1][0] - fb_l[0],
            com_pos[1][1] - fb_l[1],
            com_pos[1][2]};

        // print com position for debag
        // Serial.println("com_pos:");
        // Serial.print(com_pos[0][0]); Serial.print(", "); Serial.print(com_pos[0][1]); Serial.print(", "); Serial.println(com_pos[0][2]);

        // send order
        float phi_fb = sensor.angle_phi_fb(); // simple phi feedback
        robot->move_leg_ik(leg_right_com, com_pos[0][3], phi_fb, true);
        robot->move_leg_ik(leg_left_com, com_pos[1][3] , phi_fb, false);
        
        // arm_pos_fb
        array<float, 2> com_r = controller.rotate_vec({leg_right_com[0], leg_right_com[1]}, -com_pos[0][3]);
        array<float, 2> com_l = controller.rotate_vec({leg_left_com[0],  leg_left_com[1]},  -com_pos[1][3]);
        com_x[0] = com_r[0];
        com_x[1] = com_l[0];

        /* #########################################################################
        DELAY for NEXT CYCLE
        ##########################################################################*/
        // delay
        // vTaskDelay(pdMS_TO_TICKS(delay_duration));
        vTaskDelay(pdMS_TO_TICKS(1000.0f / CTRL_STEP));
    }
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
