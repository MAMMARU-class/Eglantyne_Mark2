#include "lower_body.h"
#include "connection.h"
#include <vector>
#include <string>
#include <cstdio>

// variables use in experiment
float selected_T_sup;
bool is_fb_on;

// orders
static array<float, 3> vd = {0.0f, 0.0f, 0.0f};

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

// body angles
// theta
float theta_order = 0.0f;
float theta = 0.0f;
float theta_err = 0.0f;
float theta_err_last = 0.0f;
// phi
float phi_order = 0.0f;
float phi = 0.0f;
float phi_err = 0.0f;
float phi_err_last = 0.0f;
// feedback
float kp_theta = 0.15f;
float kd_theta = 0.01f;
float kp_phi = 0.06f;
float kd_phi = 0.005f;

// control classes
static Robot* robot;
static MotionSD* sd;
GaitController controller;
SensorFB sensor;

void lower_body_control_init(Robot* r, MotionSD* s){
    robot = r;
    sd = s;

    sd->init();

    sensor.init(sd);
    delay(1000);
    sensor.update();
    delay(1000);
    Serial.println("Lower body control initialized");

    Serial.println("experimental setup start");

    // experimental setup
    send_msg2controller("experimental setup");
    delay(1000);
    // --- 設定値・変数の準備 ---
    // std::vector<float> T_sup_list = {0.12f, 0.14f, 0.15f, 0.16f, 0.18f, 0.2f};
    std::vector<float> T_sup_list = {0.14f, 0.18f, 0.2f, 0.25f, 0.3f, 0.35f};
    selected_T_sup = T_sup_list[0];
    is_fb_on = true;
    bool is_disturb_on = true;
    std::string created_filename = "";
    char filename_buf[64];

    // フォルダパスの設定
    const char* target_dir = "/data";

    // 1. T_sup の選択処理
    size_t t_sup_idx = 0;
    while (1) {
        // [choose] ボタン右[1]で次のリスト要素へ切り替え
        if (global_control_pkt.button_right[1] == 0) {
            t_sup_idx = (t_sup_idx + 1) % T_sup_list.size();
            while (global_control_pkt.button_right[1] == 0) {
                delay(10);
            }
            delay(5);
        }

        // コントローラへ現在の設定値を送信表示
        char msg[32];
        snprintf(msg, sizeof(msg), "T_sup: %.2f", T_sup_list[t_sup_idx]);
        send_msg2controller(msg);
        delay(5);

        // [select] ボタン右[0]で決定
        if (global_control_pkt.button_right[0] == 0) {
            selected_T_sup = T_sup_list[t_sup_idx];
            Serial.print("Selected T_sup: ");
            Serial.println(selected_T_sup);

            while (global_control_pkt.button_right[0] == 0) {
                delay(10);
            }
            delay(1000);
            break;
        }
    }

    // 2. FB (Feedback) ON/OFF の選択処理
    while (1) {
        // [choose] ボタン右[1]で ON / OFF 切り替え
        if (global_control_pkt.button_right[1] == 0) {
            is_fb_on = !is_fb_on;
            while (global_control_pkt.button_right[1] == 0) {
                delay(10);
            }
            delay(5);
        }

        // コントローラ表示
        if (is_fb_on) {
            send_msg2controller("FB: ON");
        } else {
            send_msg2controller("FB: OFF");
        }
        delay(5);

        // [select] ボタン右[0]で決定
        if (global_control_pkt.button_right[0] == 0) {
            Serial.print("Selected FB: ");
            Serial.println(is_fb_on ? "ON" : "OFF");

            while (global_control_pkt.button_right[0] == 0) {
                delay(10);
            }
            delay(1000);
            break;
        }
    }

    // 2. 外乱 (disturbance) ON/OFF の選択処理
    while (1) {
        // [choose] ボタン右[1]で ON / OFF 切り替え
        if (global_control_pkt.button_right[1] == 0) {
            is_disturb_on = !is_disturb_on;
            while (global_control_pkt.button_right[1] == 0) {
                delay(10);
            }
            delay(5);
        }

        // コントローラ表示
        if (is_disturb_on) {
            send_msg2controller("Disturbance: ON");
        } else {
            send_msg2controller("Disturbance: OFF");
        }
        delay(5);

        // [select] ボタン右[0]で決定
        if (global_control_pkt.button_right[0] == 0) {
            Serial.print("Selected Disturbance: ");
            Serial.println(is_disturb_on ? "ON" : "OFF");

            while (global_control_pkt.button_right[0] == 0) {
                delay(10);
            }
            delay(1000);
            break;
        }
    }

    // 3. 既存のファイル数をカウントして最新の試行番号 (trial_id) を算出
    int i = 0;
    while (true) {
        // 例: "/data/exp_0_T0.20_FB1.csv"
        sprintf(filename_buf, "%s/T%.2f_FB%d_dist%d_exp%03d.csv", 
                target_dir, selected_T_sup, is_fb_on ? 1 : 0, is_disturb_on ? 1 : 0, i);

        if (s->is_file_exist(filename_buf) == true) {
            i++;
        } else {
            break;
        }
    }
    created_filename = filename_buf;
    // センサクラスにファイル名をセット
    sensor.set_filename(created_filename.c_str());

    // 完了通知と状態リセット
    Serial.print("Created File Name: ");
    Serial.println(created_filename.c_str());

    send_msg2controller("LOGO");

    // initialize control parameters
    controller.init_param_walk(HEIGHT_WALK, selected_T_sup);
    controller.init_pose();

    robot->init_home(1);
    delay(500);
}

array<float, 3> update_vel(array<float, 3> vd){
    global_control_pkt.stick_right[0] = 1.0f;
    return {0.1f, 0.0f, 0.0f};
    if (!connected){
        vd = {0.0f, 0.0f, 0.0f};
        global_control_pkt.stick_right[0] = 0.0f;
        global_control_pkt.stick_right[1] = 0.0f;
        global_control_pkt.stick_left[0] = 0.0f;
        global_control_pkt.stick_left[1] = 0.0f;
        return vd;
    }
    array<float, 3> vd_max_abs = controller.get_vd_max_abs();
    vd[0] = global_control_pkt.stick_right[0] * vd_max_abs[0];
    vd[1] = -1 * global_control_pkt.stick_right[1] * vd_max_abs[1];
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
        abs(global_control_pkt.stick_right[0]) < CMD_MIN && 
        abs(global_control_pkt.stick_right[1]) < CMD_MIN && 
        abs(global_control_pkt.stick_left[1]) < CMD_MIN){
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
        array<int,3> RED    = {255, 0,   0  }; // : disconnected or free
        array<int,3> WHITE  = {255, 255, 255}; // : WAIT
        array<int,3> BLUE   = {0,   0,   255}; // : WALK
        /* ###################################################################### */
        if(!connected || mode == Mode::FREE){
            neopixelWrite(RGB_BUILTIN, RED[0], RED[1], RED[2]);
        }else if (mode == Mode::WAIT){
            neopixelWrite(RGB_BUILTIN, WHITE[0], WHITE[1], WHITE[2]);
        }else if (mode == Mode::WALK){
            neopixelWrite(RGB_BUILTIN, BLUE[0], BLUE[1], BLUE[2]);
        }else{
            neopixelWrite(RGB_BUILTIN, RED[0], RED[1], RED[2]);
        }

        /* #########################################################################
        CONTROLLER HANDLER
        Handle body angle orders.
        ######################################################################### */
        // Motion control orders
        // body angle order
        if (global_control_pkt.button_right[1] == 0){
            theta_order = BODY_ANGLE_SMALL;
        }else if (global_control_pkt.button_left[1] == 0){
            theta_order = -BODY_ANGLE_SMALL;
        }else{
            theta_order = 0.0f;
        }
        
        /* #########################################################################
        ORDER AND MODE INITIALIZEAITON
        In the first step, check
        - Torque off order is given -> free all joint and skip the rest of the loop
        - Fall                      -> Switch to FALL phase
        - Velocity update           -> Update vd with controller input and sensor feedback. Switch to WALK mode if vd is large enough
        - After fallen down (WAKE)  -> initialize parameters
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

        // update and feedback vd
        vd = update_vel(vd);
        array<float, 3> vd_fb = sensor.vd_fb(vd);
        // vd[0] += vd_fb[0];
        // vd[1] += vd_fb[1];
        // vd[2] += vd_fb[2];

        // walk if vd is large enough
        if (mode == Mode::WAIT){
            if (abs(global_control_pkt.stick_right[0]) > CMD_MIN || 
                abs(global_control_pkt.stick_right[1]) > CMD_MIN || 
                abs(global_control_pkt.stick_left[1]) > CMD_MIN){
                init_phase(
                    mode_last,
                    Phase::START,
                    0
                );
            }
        }

        // initialize parameters after fallen down
        if (phase == Phase::WAKE || (phase == Phase::WAIT && !connected)){
            theta = 0.0f;
            phi = 0.0f;
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
                    controller.init_param_walk(HEIGHT_WALK, selected_T_sup);
                    phi_order = 0.0f;
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

                // initialize pose to WALK
                controller.init_param_walk(HEIGHT_WALK, selected_T_sup);
                com_pos = controller.get_default_com_pos();
                phi_order = 0.0f;

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
        - arm position feedback
        - sensor feedback (angle)
        - sensor feedback (update rate)
        - sensor feedback (x0 and vx0)
        - START exception
        - body rotation
        ##########################################################################*/
        // arm_pos feedback
        com_x[0] = com_pos[0][0];
        com_x[1] = com_pos[1][0];

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
                is_fb_on
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

        // arm position feedback
        array<float, 3> arm_right_pos = robot->arm_k_solver({arm_right_angles[0], arm_right_angles[1], arm_right_angles[2]});
        array<float, 3> arm_left_pos  = robot->arm_k_solver({arm_left_angles[0], arm_left_angles[1], arm_left_angles[2]});
        array<float, 2> arm_mass_pos  = {arm_right_pos[0] + arm_left_pos[0], -arm_right_pos[1] + arm_left_pos[1]};
        array<float, 2> com_diff      = {arm_mass_pos[0] / 16 , arm_mass_pos[1] / 16};

        com_pos_fb[0] += com_diff[0];
        com_pos_fb[1] += com_diff[1];

        // dummy feedback
        // com_pos_fb = {0,0};
        // update_rate = UPDATE_RATE_BASE;

        // body rotation
        // theta
        theta_err = theta_order - theta;
        float theta_derr = theta_err - theta_err_last;
        theta_err_last = theta_err;
        theta += kp_theta * theta_err + kd_theta * theta_derr;
        com_pos[0][3] += theta;
        com_pos[1][3] += theta;
        // phi
        phi_err = phi_order - phi;
        float phi_derr = phi_err - phi_err_last;
        phi_err_last = phi_err;
        phi += kp_phi * phi_err + kd_phi * phi_derr;
        float l_pivot2com = sensor.get_l_pivot2com();
        com_pos_fb[0] += l_pivot2com * sin(phi);
        sensor.set_phi(phi);
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

        // send order
        float phi_fb = sensor.angle_phi_fb(); // simple phi feedback
        robot->move_leg_ik(
            leg_right_com, com_pos[0][3], 
            phi - phi_fb, -phi_fb/2, 
            true
        );
        robot->move_leg_ik(
            leg_left_com, com_pos[1][3], 
            phi - phi_fb, -phi_fb/2, 
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
