#include "lower_body.h"
#include "experimental_setup.h"
#include <string>
#include <cstdio>

const char* const LOG_COLUMN_NAMES[] = {
    "acc_x_mps2", "acc_y_mps2", "acc_z_mps2",
    "angle_x_deg", "angle_y_deg", "angle_z_deg",
    "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
    "pos_y", "update_rate_fb"
};
constexpr size_t LOG_COLUMN_COUNT =
    sizeof(LOG_COLUMN_NAMES) / sizeof(LOG_COLUMN_NAMES[0]);

// orders
static array<float, 3> vd = {0.0f, 0.0f, 0.0f};

// mode and phase
static Mode mode = Mode::WAIT;

static Phase phase = Phase::WAIT;
static Phase phase_next;
static Phase phase_last = Phase::WAIT;

// step counter
static int phase_length;
static int phase_count;
static int phase_count_x;
static int update_rate = UPDATE_RATE_BASE;
bool single_calculated = true;

// control classes
static Robot* robot;
static MotionSD* sd;
GaitController controller;
SensorFB sensor;

static void write_motion_log(bool include_feedback_values){
    BNO055Data bno_data = sensor.get_bno055_data();
    const float values[LOG_COLUMN_COUNT] = {
        bno_data.acceleration[0],
        bno_data.acceleration[1],
        bno_data.acceleration[2],
        bno_data.angle[0],
        bno_data.angle[1],
        bno_data.angle[2],
        bno_data.angular_velocity[0],
        bno_data.angular_velocity[1],
        bno_data.angular_velocity[2],
        sensor.get_last_pos_y(),
        sensor.get_last_update_rate_fb()
    };
    const bool valid[LOG_COLUMN_COUNT] = {
        true, true, true,
        true, true, true,
        true, true, true,
        include_feedback_values, include_feedback_values
    };

    sd->write_csv_row(values, valid, LOG_COLUMN_COUNT);
}

void lower_body_control_init(Robot* r, MotionSD* s){
    robot = r;
    sd = s;

    sd->init();

    sensor.init();
    delay(1000);
    sensor.update();
    delay(1000);
    Serial.println("Lower body control initialized");

    Serial.println("Experimental setup");
    Serial.print("T_sup: ");
    Serial.println(EXPERIMENT_T_SUP, 3);
    Serial.print("Feedback: ");
    Serial.println(EXPERIMENT_FB_ENABLED ? "ON" : "OFF");
    Serial.print("Disturbance trial: ");
    Serial.println(EXPERIMENT_DISTURBANCE_ENABLED ? "ON" : "OFF");

    std::string created_filename;
    char filename_buf[64];
    const char* target_dir = "/data";
    sd->create_directory(target_dir);

    int i = 0;
    while (true) {
        snprintf(filename_buf, sizeof(filename_buf), "%s/T%.2f_FB%d_dist%d_exp%03d.csv",
                 target_dir, EXPERIMENT_T_SUP, EXPERIMENT_FB_ENABLED ? 1 : 0,
                 EXPERIMENT_DISTURBANCE_ENABLED ? 1 : 0, i);

        if (s->is_file_exist(filename_buf) == true) {
            i++;
        } else {
            break;
        }
    }
    created_filename = filename_buf;

    if (!sd->begin_csv_log(
            created_filename.c_str(),
            LOG_COLUMN_NAMES,
            LOG_COLUMN_COUNT,
            EXPERIMENT_LOG_ROW_COUNT)) {
        Serial.println("Motion log initialization failed");
    }

    Serial.print("Created File Name: ");
    Serial.println(created_filename.c_str());

    // initialize control parameters
    controller.init_param_walk(HEIGHT_WALK, EXPERIMENT_T_SUP);
    controller.init_pose();

    robot->init_home(1);
    delay(500);
}

void init_phase(Mode next_mode, Phase next_phase, float next_phase_time){
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
        abs(vd[0]) < VELOCITY_EPS &&
        abs(vd[1]) < VELOCITY_EPS &&
        abs(vd[2]) < VELOCITY_EPS){
            phase_next = Phase::END;
    }else{
        phase_next = Phase::DOUBLE;
    }
}

int loop_count = 0;
void Core1Task(void * parameter){
    loop_count++;
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
        array<int,3> WHITE  = {255, 255, 255}; // : WAIT
        array<int,3> BLUE   = {0,   0,   255}; // : WALK
        /* ###################################################################### */
        if (mode == Mode::WAIT){
            neopixelWrite(RGB_BUILTIN, WHITE[0], WHITE[1], WHITE[2]);
        }else if (mode == Mode::WALK){
            neopixelWrite(RGB_BUILTIN, BLUE[0], BLUE[1], BLUE[2]);
        }

        /* #########################################################################
        ORDER AND MODE INITIALIZEAITON
        In the first step, check
        - Fall                      -> Switch to FALL phase
        - Velocity update           -> Update vd with the experiment setting. Switch to WALK mode if vd is large enough
        - After fallen down (WAKE)  -> initialize parameters
        ##########################################################################*/
        // fall check
        if(sensor.fall() && phase != Phase::FALL && phase != Phase::WAKE){
            init_phase(
                Mode::WALK,
                Phase::FALL,
                0
            );
        }

        // update and feedback vd
        vd = TARGET_VELOCITY;

        // walk if vd is large enough
        if (mode == Mode::WAIT){
            if (abs(vd[0]) > VELOCITY_EPS ||
                abs(vd[1]) > VELOCITY_EPS ||
                abs(vd[2]) > VELOCITY_EPS){
                init_phase(
                    Mode::WALK,
                    Phase::START,
                    0
                );
            }
        }

        /* #########################################################################
        TRAJECTORY CALCULATION AND PHASE UPDATE
        In the second step, calculate the desired com position based on the phase. Update the phase at the end of each phase duration
        For each phase,
        normal walking
        - START    : Initialize satrt parameters and phase length. Next phase is SINGLE.
        - END      : Initialize end parameters and phase length. Next phase is START, and change mode to WAIT.
        - SINGLE   : At the middle of the phase, decide next phase and next foot position. Next phase is (DOUBLE / END / ).
        - DOUBLE   : CoM transition between SINGLE and SINGLE. calculate next SINGLE phase parameters and change pivot in the first step. Next phase is SINGLE.
        exceptional states
        - FALL     : After slip is detected, free upper body and shrink lower body for the safety. Next phase is WAKE.
        - WAKE     : WAKE the robot up. Next phase is START. and change the mode to WAIT.
        idring
        - WAIT     : Do nothing.
        ##########################################################################*/
        // init com_pos
        array<array<float, 5>, 3> com_pos = controller.get_default_com_pos();
        // save t_ideal before phase_count updated (for update_rate feedback)
        float t_ideal = phase_count   / (float)CTRL_STEP / (float)UPDATE_RATE_BASE + controller.get_T_ds()/2;
        float tx      = phase_count_x / (float)CTRL_STEP / (float)UPDATE_RATE_BASE + controller.get_T_ds()/2;
        // phase switch-case sentences
        switch (phase){
            /* #######################################################
            normal walking
            ####################################################### */
            case Phase::START:{
                if (phase_count == 0){
                    Serial.println("phase: START");
                    controller.init_param_walk(HEIGHT_WALK, EXPERIMENT_T_SUP);
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
                    Serial.println("calculate single");
                    single_calculated = true;
                    // update state variables in gait controller
                    controller.update_state_variables(vd);
                    update_phase();
                }
                com_pos = controller.calc_com_traj_single(
                    single_calculated,
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

            /* #######################################################
            exeptional states
            ####################################################### */
            case Phase::FALL:{
                Serial.println("phase: FALL");
                sd->write_csv_null_row();
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
                sd->write_csv_null_row();
                if (sensor.face_up()){
                    wake_face_up();
                }else{
                    wake_face_down();
                }
                // phase transition
                init_phase(
                    Mode::WAIT,
                    Phase::WAIT,
                    0
                );

                // initialize pose to WALK
                controller.init_param_walk(HEIGHT_WALK, EXPERIMENT_T_SUP);
                com_pos = controller.get_default_com_pos();

                break;
            }

            /* #######################################################
            idring
            ####################################################### */
            case Phase::WAIT:{
                break;
            }
        }

        /* #########################################################################
        Skip the rest in exceptional states.
        ##########################################################################*/
        if (phase == Phase::FALL || phase == Phase::WAKE){
            // do nothing
            continue;
        }

        /* #########################################################################
        FEEDBACK
        - sensor feedback (update rate)
        - sensor feedback (x0 and vx0)
        - START exception
        ##########################################################################*/
        // update_rate feedback
        array<float, 2> acc_ideal = {com_pos[2][0], com_pos[2][1]};
        float Tc = controller.get_Tc();
        if (phase == Phase::SINGLE){
            sensor.set_update_rate_fb_gains(7.5, 0.60);
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
                com_y_pos,
                EXPERIMENT_FB_ENABLED
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
            update_rate = (int)(UPDATE_RATE_BASE / 1.2f);
        }

        // x0 and vx0 feedback
        if (phase == Phase::SINGLE){
            array<float, 2> x0_vx0 = controller.get_x0_vx0();
            // com calculation check
            float com_x_pos;
            if(controller.is_pivot_right()){
                com_x_pos = com_pos[0][0];
            }else{
                com_x_pos = com_pos[1][0];
            }

            array<float, 2> x0_vx0_fb = sensor.x0_vx0_fb(
                tx,
                x0_vx0[0], x0_vx0[1],
                Tc, CTRL_STEP,
                com_x_pos
            );
            controller.feedback_x0_vx0(x0_vx0_fb);
        }

        // Record BNO055 data throughout normal walking. Feedback values are
        // meaningful only during single support.
        if (phase == Phase::SINGLE){
            write_motion_log(true);
        }else if (phase == Phase::DOUBLE){
            write_motion_log(false);
        }

        /* #########################################################################
        EXECUTION
        - move robot
        - delay
        ##########################################################################*/
        // move robot
        // devide com pos into each leg
        array<float, 3> leg_right_com = {
            com_pos[0][0],
            com_pos[0][1],
            com_pos[0][2]};
        array<float, 3> leg_left_com =  {
            com_pos[1][0],
            com_pos[1][1],
            com_pos[1][2]};

        // send order
        float phi_fb = sensor.angle_phi_fb(); // simple phi feedback
        robot->move_leg_ik(
            leg_right_com, com_pos[0][3], 
            -phi_fb, -phi_fb/2,
            true
        );
        robot->move_leg_ik(
            leg_left_com, com_pos[1][3], 
            -phi_fb, -phi_fb/2,
            false
        );
        
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

    sd->play_motion(robot, "/wake_face_up.csv", 0.15f);
    robot->init_home(1);
}

void wake_face_down(){
    Serial.println("Wake face down");

    sd->play_motion(robot, "/wake_face_down.csv", 0.15f);
    robot->init_home(1);
}
