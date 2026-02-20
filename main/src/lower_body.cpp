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
GaitController controller;
SensorFB sensor;

void lower_body_control_init(Robot* r){
    robot = r;
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
    while(1) {
        // torque off order
        // fall check
        if(sensor.fall()){
            phse = Phase::FALL;
        }

        // vd update
        vd = update_vel(vd);
        // array<float, 3> vd_fb = sensor.vd_fb(vd);
        // vd[0] += vd_fb[0];
        // vd[1] += vd_fb[1];
        // vd[2] += vd_fb[2];

        array<array<float, 5>, 3> com_pos;

        switch (phase){
            case Phase::START:{
                if (phase_count == 0){
                    Serial.println("phase: START");
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
                

                // phase transition
                init_phase(
                    Mode::WAIT,
                    Phase::WAKE,
                    0,
                )
            }
        }

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
        
        // move robot
        // devide com pos into each leg
        array<float, 3> leg_right_com = {com_pos[0][0] - com_pos_fb[0], com_pos[0][1] - com_pos_fb[1], com_pos[0][2]};
        array<float, 3> leg_left_com =  {com_pos[1][0] - com_pos_fb[0], com_pos[1][1] - com_pos_fb[1], com_pos[1][2]};
        // check com pos
        // Serial.print("com right: "); Serial.print(leg_right_com[0], 4); Serial.print(", "); Serial.print(leg_right_com[1], 4); Serial.print(", "); Serial.println(leg_right_com[2], 4);
        // Serial.print("com left: ");  Serial.print(leg_left_com[0]); Serial.print(", "); Serial.print(leg_left_com[1]); Serial.print(", "); Serial.println(leg_left_com[2]);
        // move legs
        robot->move_leg_ik(leg_right_com, com_pos[0][3], 0.0, true);
        robot->move_leg_ik(leg_left_com, com_pos[1][3], 0.0, false);

        // robot->move_leg_ik({- com_pos_fb[0], 0.04 - com_pos_fb[1], 0.158}, com_pos[0][3], 0.0, true);
        // robot->move_leg_ik({- com_pos_fb[0], -0.04 - com_pos_fb[1], 0.158}, com_pos[1][3], 0.0, false);

        // delay
        vTaskDelay(pdMS_TO_TICKS(delay_duration));
    }
}
