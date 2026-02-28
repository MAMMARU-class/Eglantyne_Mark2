#ifndef MSG_H
#define MSG_H

# include <stdint.h>

typedef struct __attribute__((packed)) {
    int32_t button_right[3];
    float   stick_right[3];
    float   arm_right[3];
    int32_t button_left[3];
    float   stick_left[3];
    float   arm_left[3];
} ControlPacket;

// controller peripheral variables
extern bool connected;
extern bool order_free;
extern volatile ControlPacket global_control_pkt;
// feedback variables
extern std::array<float, 2> com_x;
extern std::array<float, 3> arm_right_angles;
extern std::array<float, 3> arm_left_angles;

#endif
