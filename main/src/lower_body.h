#include "Robot.h"
#include "GaitController.h"
#include "SensorFB.h"

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
    FLIGHT
};

void lower_body_control_init(Robot* r);

void init_phase(Mode next_mode, Phase next_phase, float next_phase_length);
Phase update_phase();
void Core1Task(void * parameter);
