#ifndef LOWER_BODY_H
#define LOWER_BODY_H

#include "Robot.h"
#include "MotionSD.h"
#include "GaitController.h"
#include "SensorFB.h"
#include "pinassign.h"

// #define CTRL_STEP 100 //Hz
#define CTRL_STEP 130 //Hz
#define UPDATE_RATE_BASE 100 // step

enum class Mode: uint8_t {
    // exceptional states
    WAIT,
    WALK
};

enum class Phase: uint8_t {
    // normal walking
    START,
    END,
    SINGLE,
    DOUBLE,
    // exceptional states
    FALL,
    WAKE,
    // idring
    WAIT
};

void lower_body_control_init(Robot* r, MotionSD* s);
bool lower_body_load_experiment_config();
void set_target_yaw_deg(float yaw_target_deg);

void init_phase(Mode next_mode, Phase next_phase, float next_phase_length);
void update_phase();

void Core1Task(void * parameter);

void wake_face_up();
void wake_face_down();

#endif
