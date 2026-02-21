#include "Robot.h"
#include "MotionSD.h"
#include "GaitController.h"
#include "SensorFB.h"
#include "msg.h"
#include "pinassign.h"

#define CTRL_STEP 100 //Hz

enum class Mode: uint8_t {
    WAIT,
    WALK
};

enum class Phase: uint8_t {
    START,
    END,
    SINGLE,
    DOUBLE,
    FLIGHT,
    FALL,
    WAKE
};

void lower_body_control_init(Robot* r, MotionSD* s);

void init_phase(Mode next_mode, Phase next_phase, float next_phase_length);
Phase next_phase();
void Core1Task(void * parameter);

void wake_face_up();
void wake_face_down();
