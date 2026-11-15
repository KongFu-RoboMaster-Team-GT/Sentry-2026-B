#ifndef TEST_CONTROL_H
#define TEST_CONTROL_H
#include <stdint.h>
#include <stddef.h>
typedef struct {
    uint16_t ch0, ch1, ch2, ch3;
    uint8_t sw, pause, btn_l, btn_r;
    uint16_t dial;
    uint8_t trigger;
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_btn_l, mouse_btn_r, mouse_btn_m;
    uint16_t key_w, key_s, key_a, key_d, key_shift, key_ctrl;
    uint16_t key_q, key_e, key_r, key_f, key_g, key_z;
    uint16_t key_x, key_c, key_v, key_b, crc;
} VTM_Data_t;
extern VTM_Data_t key;
typedef struct { uint8_t flag; } test_calibrate_t;
extern test_calibrate_t calibrate;
uint32_t osKernelSysTick(void);
void osDelay(uint32_t ticks);
void osDelayUntil(uint32_t *wake, uint32_t period);
#endif
