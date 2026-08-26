#ifndef LOWER_BODY_H
#define LOWER_BODY_H

#include "Robot.h"
#include "MotionSD.h"
#include "GaitController.h"
#include "SensorFB.h"
#include "msg.h"
#include "pinassign.h"

// #define CTRL_STEP 100 //Hz
#define CTRL_STEP 130 //Hz
#define UPDATE_RATE_BASE 100 // step

// orders
#define CMD_MIN 0.15f
#define BODY_ANGLE_SMALL 25.0f * PI / 180.0f

// rotation feedback gains
#define KP_THETA_BASE 0.15f
#define KD_THETA_BASE 0.01f
#define KP_PHI_BASE 0.06f
#define KD_PHI_BASE 0.005f

enum class Mode: uint8_t {
    // exceptional states
    WAIT,
    FREE,
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

typedef struct __attribute__((packed)) {
    float height_diff;
    float relative_body_angle;
    float relative_body_pos;
    float relative_leg_angle;
} STANCE_INFO;

void lower_body_control_init(Robot* r, MotionSD* s);

array<float, 3> update_vel(array<float, 3> vd);
void init_phase(Mode next_mode, Phase next_phase, float next_phase_length);
void update_phase();
array<array<float, 5>, 3> attach_stance(array<array<float, 5>, 3> com_pos, STANCE_INFO stance);

void Core1Task(void * parameter);

array<float, 2> body_angle_fb();
void wake_face_up();
void wake_face_down();

#endif
