# include <stdint.h>

typedef struct __attribute__((packed)) {
    int32_t button_right[3];
    float   stick_right[3];
    float   arm_right[3];
    int32_t button_left[3];
    float   stick_left[3];
    float   arm_left[3];
} ControlPacket;

extern volatile ControlPacket global_control_pkt;
