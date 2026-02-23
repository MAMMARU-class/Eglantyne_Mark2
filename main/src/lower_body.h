#ifndef LOWER_BODY_H
#define LOWER_BODY_H

#include "Robot.h"
#include "MotionSD.h"
#include "GaitController.h"
#include "SensorFB.h"
#include "msg.h"
#include "pinassign.h"

#define CTRL_STEP 100 //Hz

enum class Order: uint8_t {
    NONE,
    MODE_CHANGE,
    ROTATE,
}

enum class Mode: uint8_t {
    WAIT,
    TRANSITION,
    WALK,
    FIGHT
};

enum class Phase: uint8_t {
    START,
    END,
    SINGLE,
    DOUBLE,
    STANCE,
    FLIGHT,
    FALL,
    WAKE
};

typedef struct __attribute__((packed)) {
    float relative_body_angle;
    float relative_body_pos;
    float relative_leg_angle;
} STANCE_INFO;


void lower_body_control_init(Robot* r, MotionSD* s);

void init_phase(Mode next_mode, Phase next_phase, float next_phase_length);
void update_phase();
void Core1Task(void * parameter);

void wake_face_up();
void wake_face_down();

#endif
