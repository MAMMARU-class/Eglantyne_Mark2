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

// orders
#define VD_MIN 0.01f
#define BODY_ANGLE_SMALL 15.0f * PI / 180.0f
#define BODY_ANGLE_LARGE 45.0f * PI / 180.0f

enum class Order: uint8_t {
    NONE,

    // basic orders
    MODE_CHANGE,
    GUARD,

    // orders while WALK mode
    CROUCH,
    JUMP,
    RUN,

    // orders while CROUCH mode
    STAND,
    LEARN,
    THROW,
    ROLL,
    
    // orders while FIGHT mode
    KICK_LOW,
    KICK_MIDDLE,
    KICK_BACK
};

enum class Mode: uint8_t {
    // exceptional states
    WAIT,
    FREE,
    TRANSITION,
    MOTION_PLAY,

    // normal states
    WALK,
    CROUCH,
    FIGHT
};

enum class Phase: uint8_t {
    // normal walking
    START,
    END,
    SINGLE,
    DOUBLE,
    // stance change
    STANCE,
    // havent decided
    FLIGHT,
    // exceptional states
    FALL,
    WAKE,
    // order while WALK
    JUMP,
    RUN,
    // order while CROUCH
    LEARN,
    THROUGH,
    ROLL,
    // order while FIGHT
    GUARD,
    KICK_LOW,
    KICK_MIDDLE,
    KICK_BACK,
    // idring
    WAIT
};

typedef struct __attribute__((packed)) {
    float height_diff;
    float relative_body_angle;
    float relative_body_pos;
    float relative_leg_angle;
} STANCE_INFO;

// order related variables
enum class JumpState : uint8_t{
    CROUCH,
    EXTEND,
    FLY,
    HIT
};

void lower_body_control_init(Robot* r, MotionSD* s);

array<float, 3> update_vel(array<float, 3> vd, Order order);
void init_phase(Mode next_mode, Phase next_phase, float next_phase_length);
void update_phase();
array<array<float, 5>, 3> attach_stance(array<array<float, 5>, 3> com_pos, STANCE_INFO stance);
STANCE_INFO update_stance_diff(
    STANCE_INFO stance_next, int phase_length,
    float height_aim, float height_now,
    Mode mode_next);

void Core1Task(void * parameter);

array<float, 2> body_angle_fb();
void wake_face_up();
void wake_face_down();

#endif
