#ifndef LOWER_BODY_H
#define LOWER_BODY_H

#include "Robot.h"
#include "MotionSD.h"
#include "GaitController.h"
#include "SensorFB.h"
#include "msg.h"
#include "pinassign.h"

#define CTRL_STEP 100 //Hz
#define UPDATE_RATE_BASE 100 // step

#define VD_MIN 0.01f

enum class Order: uint8_t {
    NONE,

    // basic orders
    MODE_CHANGE,

    // orders while WALK mode
    CROUCH,
    UNCROUCH

    // orders while FIGHT mode
};

enum class Mode: uint8_t {
    // exceptional states
    WAIT,
    FREE,
    TRANSITION,

    // normal states
    WALK,
    FIGHT,
    CROUCH
};

enum class Phase: uint8_t {
    STANCE,
    START,
    END,
    SINGLE,
    DOUBLE,
    FLIGHT,
    FALL,
    WAKE,
    WAIT
};

typedef struct __attribute__((packed)) {
    float height_diff;
    float relative_body_angle;
    float relative_body_pos;
    float relative_leg_angle;
} STANCE_INFO;


void lower_body_control_init(Robot* r, MotionSD* s);

array<float, 3> update_vel(array<float, 3> vd, Order order);
void init_phase(Mode next_mode, Phase next_phase, float next_phase_length);
void update_phase();
array<array<float, 5>, 3> attach_stance(array<array<float, 5>, 3> com_pos, STANCE_INFO stance);

void Core1Task(void * parameter);

void crouch(Order order, array<array<float, 5>, 3>& com_pos);
void wake_face_up();
void wake_face_down();

#endif
