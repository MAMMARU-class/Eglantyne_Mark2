#include "lower_body.h"
#include "experimental_setup.h"
#include "ExperimentConfig.h"
#include "ExperimentConfigLoader.h"
#include "velocity_control.h"
#include <string>
#include <cstdio>

const char* const LOG_COLUMN_NAMES[] = {
    "acc_x_mps2", "acc_y_mps2", "acc_z_mps2",
    "angle_x_deg", "angle_y_deg", "angle_z_deg",
    "gyro_x_rps", "gyro_y_rps", "gyro_z_rps",
    "t_sup_s", "pos_y", "update_rate_fb",
    "experiment_content", "update_rate_kp", "update_rate_kd"
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
VelocityControl velocity_control;
ExperimentConfig experiment_config;
ExperimentConfigLoader experiment_config_loader;
ExperimentManager experiment_manager;
static UpdateRateFeedbackGains current_update_rate_fb_gains = {0.0f, 0.0f};
static bool experiment_log_save_attempted = false;
static bool experiment_log_save_succeeded = false;
static bool motion_sd_initialized = false;
static bool experiment_config_loaded = false;

static UpdateRateFeedbackGains calculate_experiment_fb_gains(
    float t_sup)
{
    return experiment_config.calculate_update_rate_gains(t_sup);
}

static void apply_update_rate_fb_gains(float t_sup){
    current_update_rate_fb_gains = calculate_experiment_fb_gains(t_sup);
    sensor.set_update_rate_fb_gains(
        current_update_rate_fb_gains.kp,
        current_update_rate_fb_gains.kd);
}

static void apply_current_experiment_condition(){
    const float t_sup = experiment_manager.get_t_sup();
    controller.set_T_sup(t_sup);
    apply_update_rate_fb_gains(t_sup);
}

void set_target_yaw_deg(float yaw_target_deg){
    velocity_control.set_yaw_target_deg(yaw_target_deg);
    velocity_control.reset_yaw_feedback();
}

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
        controller.get_T_sup(),
        sensor.get_last_pos_y(),
        sensor.get_last_update_rate_fb(),
        static_cast<float>(
            static_cast<uint8_t>(experiment_manager.get_content())),
        current_update_rate_fb_gains.kp,
        current_update_rate_fb_gains.kd
    };
    const bool valid[LOG_COLUMN_COUNT] = {
        true, true, true,
        true, true, true,
        true, true, true,
        true,
        include_feedback_values, include_feedback_values,
        true, true, true
    };

    sd->write_csv_row(values, valid, LOG_COLUMN_COUNT);
}

void lower_body_control_init(Robot* r, MotionSD* s){
    robot = r;
    sd = s;
    experiment_config_loaded = false;
    experiment_log_save_attempted = false;
    experiment_log_save_succeeded = false;

    motion_sd_initialized = sd->init();

    sensor.init();
    delay(1000);
    sensor.update();
    delay(1000);
    Serial.println("Lower body hardware initialized");

    robot->init_home(1);
    delay(500);
}

bool lower_body_load_experiment_config(){
    if (robot == nullptr || sd == nullptr) {
        Serial.println("Experiment configuration error: control objects are null");
        return false;
    }
    if (!motion_sd_initialized) {
        Serial.println("Experiment configuration error: SD card is unavailable");
        return false;
    }

    ExperimentConfig loaded_config;
    if (!experiment_config_loader.load(
            loaded_config,
            EXPERIMENT_OPTIONS_PATH,
            EXPERIMENT_UPDATE_RATE_GAIN_PATH,
            EXPERIMENT_PROCEDURE_PATH)) {
        Serial.print("Experiment configuration error: ");
        Serial.println(experiment_config_loader.error_message());
        return false;
    }

    experiment_config = loaded_config;
    experiment_manager.configure(
        experiment_config.procedure.data(),
        experiment_config.procedure_count);

    Serial.println("Experimental setup loaded from SD card");
    Serial.print("Feedback gain mode: ");
    Serial.println(feedback_gain_mode_name(
        experiment_config.feedback_gain_mode));
    Serial.println("Update-rate feedback gain table:");
    for (size_t gain_index = 0;
         gain_index < experiment_config.update_rate_gain_count;
         ++gain_index) {
        const UpdateRateGainPoint& point =
            experiment_config.update_rate_gain_table[gain_index];
        Serial.print("  T_sup=");
        Serial.print(point.t_sup, 4);
        Serial.print(", Kp=");
        Serial.print(point.kp, 6);
        Serial.print(", Kd=");
        Serial.println(point.kd, 6);
    }
    Serial.println("Experimental procedure:");
    for (size_t procedure_index = 0;
         procedure_index < experiment_config.procedure_count;
         ++procedure_index) {
        const ExperimentProcedureItem& item =
            experiment_config.procedure[procedure_index];
        Serial.print("  #");
        Serial.print(procedure_index);
        Serial.print(": T_sup=");
        Serial.print(item.t_sup_start, 3);
        Serial.print(" -> ");
        Serial.print(item.t_sup_end, 3);
        Serial.print(", steps=");
        Serial.print(item.step_count);
        Serial.print(", on_error=");
        Serial.print(error_action_name(item.error_action));
        Serial.print(", content=");
        Serial.println(experiment_content_name(item.content));
    }
    Serial.print("Fixed x0/vx0 gains: Kp=");
    Serial.print(experiment_config.x0_vx0_gains.kp, 6);
    Serial.print(", Kd=");
    Serial.println(experiment_config.x0_vx0_gains.kd, 6);
    Serial.print("Disturbance trial: ");
    Serial.println(disturbance_type_name(
        experiment_config.disturbance_type));
    Serial.print("Log row count: ");
    Serial.println(experiment_config.log_row_count);

    char experiment_name[80];
    char filename_prefix[96];
    char config_copy_prefix[128];
    char filename_buf[112];
    const char* target_dir = "/data";
    const char* config_copy_dir = "/data/experiment_config";
    sd->create_directory(target_dir);
    sd->create_directory(config_copy_dir);

    int i = 0;
    while (true) {
        snprintf(
            experiment_name,
            sizeof(experiment_name),
            "T%.2f-%.2f_gain-%s_dist-%s_exp%03d",
            experiment_config.procedure[0].t_sup_start,
            experiment_config.procedure[
                experiment_config.procedure_count - 1].t_sup_end,
            feedback_gain_mode_name(
                experiment_config.feedback_gain_mode),
            disturbance_type_name(experiment_config.disturbance_type),
            i);
        snprintf(
            filename_prefix,
            sizeof(filename_prefix),
            "%s/%s",
            target_dir,
            experiment_name);
        snprintf(
            filename_buf,
            sizeof(filename_buf),
            "%s.csv",
            filename_prefix);

        if (sd->is_file_exist(filename_buf)) {
            i++;
        } else {
            break;
        }
    }

    snprintf(
        config_copy_prefix,
        sizeof(config_copy_prefix),
        "%s/%s",
        config_copy_dir,
        experiment_name);
    if (!experiment_config_loader.copy_loaded_files(config_copy_prefix)) {
        Serial.print("Experiment configuration copy error: ");
        Serial.println(experiment_config_loader.error_message());
        return false;
    }

    if (!sd->begin_csv_log(
            filename_buf,
            LOG_COLUMN_NAMES,
            LOG_COLUMN_COUNT,
            experiment_config.log_row_count)) {
        Serial.println("Motion log initialization failed");
        return false;
    }

    apply_update_rate_fb_gains(experiment_manager.get_t_sup());
    sensor.set_x0_vx0_fb_gains(
        experiment_config.x0_vx0_gains.kp,
        experiment_config.x0_vx0_gains.kd);

    // target_velocity_x/y specify vd_x/y at T_sup = 0.14 s.
    velocity_control.set_x_velocity_at_reference(
        experiment_config.target_velocity[0]);
    velocity_control.set_y_velocity_at_reference(
        experiment_config.target_velocity[1]);
    // Keep the yaw command at zero until an external command source is added.
    set_target_yaw_deg(0.0f);

    Serial.print("Created File Name: ");
    Serial.println(filename_buf);

    // initialize control parameters
    controller.init_param_walk(HEIGHT_WALK, experiment_manager.get_t_sup());
    controller.init_pose();
    experiment_config_loaded = true;
    return true;
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
        abs(vd[0]) < experiment_config.velocity_eps &&
        abs(vd[1]) < experiment_config.velocity_eps &&
        abs(vd[2]) < experiment_config.velocity_eps){
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
    if(!experiment_config_loaded){
        Serial.println("Experiment configuration is not loaded");
        while(1);
    }
    Serial.println("Core1Task started");

    while(1) {
        sensor.update();
        /* #########################################################################
        LED HANDLER */
        array<int,3> BLUE   = {0,   0,   255}; // : WALK content
        array<int,3> YELLOW = {255, 255, 0};   // : SAVING LOG
        array<int,3> GREEN  = {0,   255, 0};   // : other content / LOG SAVED
        array<int,3> RED    = {255, 0,   0};   // : LOG SAVE FAILED
        /* ###################################################################### */
        if (experiment_log_save_attempted){
            const array<int,3>& result_color =
                experiment_log_save_succeeded ? GREEN : RED;
            neopixelWrite(
                RGB_BUILTIN,
                result_color[0], result_color[1], result_color[2]);
        }else if (experiment_manager.is_running() &&
                  experiment_manager.get_content() ==
                      ExperimentContent::WALK){
            neopixelWrite(RGB_BUILTIN, BLUE[0], BLUE[1], BLUE[2]);
        }else{
            neopixelWrite(RGB_BUILTIN, GREEN[0], GREEN[1], GREEN[2]);
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
            if (experiment_manager.is_running()){
                const ErrorAction fall_action =
                    experiment_manager.get_error_action();
                experiment_manager.on_fall();
                Serial.print("Experiment fall action: ");
                Serial.println(error_action_name(fall_action));
            }
            init_phase(
                Mode::WALK,
                Phase::FALL,
                0
            );
        }

        // update and feedback vd
        if (experiment_manager.is_running()){
            vd = experiment_config.target_velocity;
            vd[0] = velocity_control.calculate_x_velocity(
                controller.get_T_sup());
            vd[1] = velocity_control.calculate_y_velocity(
                controller.get_T_sup());

            const BNO055Data bno_data = sensor.get_bno055_data();
            vd[2] = velocity_control.calculate_yaw_velocity(
                bno_data.angle[0], 1.0f / static_cast<float>(CTRL_STEP));
        }else{
            vd = {0.0f, 0.0f, 0.0f};
            velocity_control.reset_yaw_feedback();
        }

        // walk if vd is large enough
        if (mode == Mode::WAIT){
            if (abs(vd[0]) > experiment_config.velocity_eps ||
                abs(vd[1]) > experiment_config.velocity_eps ||
                abs(vd[2]) > experiment_config.velocity_eps){
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
                    controller.init_param_walk(
                        HEIGHT_WALK,
                        experiment_manager.get_t_sup());
                    apply_update_rate_fb_gains(
                        experiment_manager.get_t_sup());
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
                    if (phase_next == Phase::DOUBLE &&
                        experiment_manager.is_running()){
                        const bool procedure_changed =
                            experiment_manager.on_step_completed();
                        if (procedure_changed){
                            if (experiment_manager.is_end()){
                                Serial.println("Experiment procedure: END");
                            }else{
                                Serial.print("Experiment procedure index: ");
                                Serial.print(
                                    experiment_manager.get_procedure_index());
                                Serial.print(", content: ");
                                Serial.print(experiment_content_name(
                                    experiment_manager.get_content()));
                                Serial.print(", T_sup: ");
                                Serial.println(
                                    experiment_manager.get_t_sup(), 4);
                            }
                        }
                    }

                    if (experiment_manager.is_end()){
                        vd = {0.0f, 0.0f, 0.0f};
                        init_phase(mode, Phase::END, 0);
                    }else{
                        init_phase(
                            mode,
                            phase_next,
                            controller.get_T_ds()
                        );
                    }
                }
                break;
            }

            case Phase::DOUBLE:{
                if (phase_count == 0){
                    Serial.println("phase: DOUBLE");
                    controller.inverse_pivot();

                    // Keep T_sup fixed during single support. Update it only
                    // after the completed step has entered double support.
                    apply_current_experiment_condition();
                    controller.init_state_variables(false, false);
                    phase_length =
                        controller.get_T_ds() * CTRL_STEP * UPDATE_RATE_BASE;
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
                experiment_manager.on_recovery_completed();
                controller.init_param_walk(
                    HEIGHT_WALK,
                    experiment_manager.get_t_sup());
                apply_update_rate_fb_gains(
                    experiment_manager.get_t_sup());
                com_pos = controller.get_default_com_pos();

                break;
            }

            /* #######################################################
            idring
            ####################################################### */
            case Phase::WAIT:{
                if (experiment_manager.is_end() &&
                    !experiment_log_save_attempted){
                    Serial.println("Saving experiment log...");
                    neopixelWrite(
                        RGB_BUILTIN,
                        YELLOW[0], YELLOW[1], YELLOW[2]);

                    experiment_log_save_succeeded =
                        sd->finish_csv_log();
                    experiment_log_save_attempted = true;

                    if (experiment_log_save_succeeded){
                        Serial.println("Experiment log saved");
                        neopixelWrite(
                            RGB_BUILTIN,
                            GREEN[0], GREEN[1], GREEN[2]);
                    }else{
                        Serial.println("Experiment log save failed");
                        neopixelWrite(
                            RGB_BUILTIN,
                            RED[0], RED[1], RED[2]);
                    }
                }
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
        controller.update_T_sup_x(float(1/CTRL_STEP)* (UPDATE_RATE_BASE - update_rate)/UPDATE_RATE_BASE);

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
