#ifndef __PROTOCOL_H
#define __PROTOCOL_H
#include "main.h"
#include <stdbool.h>

#define PC_COMMAND_TIMEOUT_MS 200u
#define PC_PARTIAL_TIMEOUT_MS 50u
#define PC_REPLY_MAX 19u
#define PC_RX_CAPACITY 512u
#define PC_REPLY_CAPACITY 32u

typedef struct { float deltaYaw, deltaPitch; bool auto_fire_flag; } usb_t;
typedef struct { uint8_t vx_detect, vy_detect, omega_detect, auto_fire_detect; } motion_t;
typedef struct {
    int32_t health, fire_number;
    float heat;
    uint8_t zone_is_ours, stop;
} pc_referee_status_t;
typedef struct {
    uint32_t rx_bytes, valid_frames, checksum_errors, format_errors;
    uint32_t partial_timeouts, rx_overflows, rx_expired, tx_dropped, tx_queued, tx_sent;
    uint32_t aim_commands, stop_commands, command_timeouts, link_resets;
} pc_diag_t;
extern usb_t usb;
extern motion_t motion;
extern volatile pc_diag_t pc_diag;

/* ISR copies bytes; parsing/replies run in USBRECEIVE_TASK. */
void pc_receive_isr(const uint8_t *data, uint16_t len);
void pc_link_reset_isr(void);
void pc_process_rx(void);
uint8_t usbreceive(const uint8_t *data, uint16_t len);
void pc_poll(void);
uint8_t pc_tx_peek(uint8_t **data, uint16_t *len);
void pc_tx_pop(void);
void pc_set_referee_status(const pc_referee_status_t *status);
void pc_note_attitude(float measured_yaw_deg);
void pc_note_fire(uint8_t enabled);
uint8_t pc_stop_active(void);
uint8_t pc_fire_requested(void);
uint8_t pc_take_aim(float *yaw_deg, float *pitch_deg);
void pc_clear_command(void);
#endif
