#include "lower_body.h"

static array<float, 3> vd = {0.15f, 0.0f, 0.0f};

static Mode mode = Mode::WALK;

static Phase phase = Phase::START;
static Phase phase_next;

// step counter
static int phase_length;
static int phase_count;

static Robot* robot;
GaitController controller;

void lower_body_control_init(Robot* r){
    robot = r;
    Serial.println("Lower body control initialized");
}

void update_vel(){
    vd = vd;
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
        switch (phase){
            case Phase::START:{
                if (phase_count == 0){
                    Serial.println("phase: START");
                    controller.init_param_walk();
                    controller.init_start();
                    float T_ds = controller.get_T_sup() * controller.get_ds_ratio() * 0.5f;
                    phase_length = T_ds * CTRL_STEP;
                }

                array<array<float, 5>, 2> com_pos = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP);
                array<float, 3> leg_right_com = {com_pos[0][0], com_pos[0][1], com_pos[0][2]};
                array<float, 3> leg_left_com = {com_pos[1][0], com_pos[1][1], com_pos[1][2]};

                Serial.print("com right: "); Serial.print(leg_right_com[0], 4); Serial.print(", "); Serial.print(leg_right_com[1], 4); Serial.print(", "); Serial.println(leg_right_com[2], 4);
                // Serial.print("com left: ");  Serial.print(leg_left_com[0]); Serial.print(", "); Serial.print(leg_left_com[1]); Serial.print(", "); Serial.println(leg_left_com[2]);

                robot->move_leg_ik(leg_right_com, com_pos[0][3], 0.0, true);
                robot->move_leg_ik(leg_left_com, com_pos[1][3], 0.0, false);

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
                    update_vel();
                    controller.update_state_variables(vd);
                    controller.init_single_half();
                    phase_next = update_phase();
                }

                array<array<float, 5>, 2> com_pos_single = controller.calc_com_traj_single(phase_count / (float)CTRL_STEP);
                array<float, 3> leg_right_com = {com_pos_single[0][0], com_pos_single[0][1], com_pos_single[0][2]};
                array<float, 3> leg_left_com = {com_pos_single[1][0], com_pos_single[1][1], com_pos_single[1][2]};

                Serial.print("com right: "); Serial.print(leg_right_com[0], 4); Serial.print(", "); Serial.print(leg_right_com[1], 4); Serial.print(", "); Serial.println(leg_right_com[2], 4);
                // Serial.print("com left: ");  Serial.print(leg_left_com[0]); Serial.print(", "); Serial.print(leg_left_com[1]); Serial.print(", "); Serial.println(leg_left_com[2]);

                robot->move_leg_ik(leg_right_com, com_pos_single[0][3], 0.0, true);
                robot->move_leg_ik(leg_left_com, com_pos_single[1][3], 0.0, false);

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
                array<array<float, 5>, 2> com_pos_double = controller.calc_com_traj_double(phase_count / (float)CTRL_STEP);
                array<float, 3> leg_right_com = {com_pos_double[0][0], com_pos_double[0][1], com_pos_double[0][2]};
                array<float, 3> leg_left_com = {com_pos_double[1][0], com_pos_double[1][1], com_pos_double[1][2]};
                
                Serial.print("com right: "); Serial.print(leg_right_com[0], 4); Serial.print(", "); Serial.print(leg_right_com[1], 4); Serial.print(", "); Serial.println(leg_right_com[2], 4);
                // Serial.print("com left: ");  Serial.print(leg_left_com[0]); Serial.print(", "); Serial.print(leg_left_com[1]); Serial.print(", "); Serial.println(leg_left_com[2]);

                robot->move_leg_ik(leg_right_com, com_pos_double[0][3], 0.0, true);
                robot->move_leg_ik(leg_left_com, com_pos_double[1][3], 0.0, false);

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
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / CTRL_STEP));
    }
}
