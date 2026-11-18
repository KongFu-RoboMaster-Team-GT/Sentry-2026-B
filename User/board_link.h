#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include "board_config.h"
#include "can.h"
#include "control_task.h"

#define BOARD_CHASSIS_INPUT_ID 0x320u
#define BOARD_YAW_COMMAND_ID 0x321u
#define BOARD_YAW_FEEDBACK_ID 0x324u
#define BOARD_STATUS_ID 0x32Fu

/* Read-only diagnostics for Keil Watch. Queued TX is not proof of bus delivery. */
typedef enum {
    BOARD_CHASSIS_ACCEPTED = 0,
    BOARD_CHASSIS_BAD_VERSION = 1,
    BOARD_CHASSIS_BAD_CHECKSUM = 2,
    BOARD_CHASSIS_BAD_CHANNEL = 3,
    BOARD_CHASSIS_OLD_SEQUENCE = 4
} board_chassis_reject_t;

typedef struct {
    uint32_t busoff_count;
    uint32_t start_retry_count;
    uint32_t recovered_count;
    uint32_t stage; /* 0=normal, 1=waiting for init ACK, 2=waiting for bus recovery */
    uint32_t last_error_esr;
    uint32_t last_hal_error;
    uint32_t abort_fail_count;
    uint32_t notify_fail_count;
} board_can_diag_t;

typedef struct {
    uint32_t role;
    uint32_t protocol_version;
    uint32_t poll_count;
    uint32_t can1_rx_count;
    uint32_t last_can1_id;
    uint32_t can1_invalid_frame_count;
    uint32_t can1_rx_error_count;
    uint32_t chassis_frame_count;
    uint32_t chassis_rx_version;
    uint32_t chassis_reject_reason;
    uint32_t chassis_tx_queued_count;
    uint32_t can1_esr;
    uint32_t can1_hal_error;
    board_can_diag_t can1;
    board_can_diag_t can2;
    /* UPPER CAN1 motor feedback only: indices are feedback ID minus 0x201.
     * Includes friction 0/1, trigger 2 and pitch 5; never board-link frames. */
    uint32_t motor_can1_rx_count[7];
    uint32_t motor_can1_last_tick_ms[7];
    uint32_t motor_can1_age_ms[7];
    uint32_t shooter_tx_queued_count, shooter_tx_fail_count;
#if BOARD_ROLE == BOARD_UPPER
    uint32_t chassis_tx_fail_count, chassis_tx_last_tick_ms, chassis_tx_max_gap_ms;
    uint32_t can1_tx_expired_count, can1_tx_abort_fail_count;
    uint32_t poll_max_gap_ms, can1_mcr, can1_tsr;
#endif
} board_link_diag_t;

extern volatile board_link_diag_t board_diag;

#if BOARD_ROLE == BOARD_LOWER
/* LOWER output path. A queued frame is not an acknowledged transmission. */
typedef struct {
    uint32_t chassis_loop_count;
    uint16_t ch0, ch1;
    uint8_t mode, motion_enabled, yaw_enabled;
    int16_t chassis_command[4];
    int16_t yaw_received, yaw_command;
    uint32_t can2_rx_count, last_can2_id;
    uint32_t motor_rx_count[5];
    uint32_t can2_tx_queued_count, can2_tx_fail_count;
    uint32_t can2_rx_error_count, can2_invalid_frame_count;
    uint32_t can2_state, can2_error;
    uint32_t can2_mcr, can2_msr, can2_tsr, can2_esr;
    /* UINT32_MAX means no feedback yet. Saturation counts are per 1ms loop. */
    uint32_t feedback_age_ms[5];
    uint32_t wheel_saturation_count[4];
    uint8_t stale_wheel_mask, braking;
    uint8_t idle_latched;
    uint32_t brake_timeout_count;
    /* Read-only stutter diagnosis. Input reason: 0=ready, 1=never seen,
     * 2=UPPER disabled input, 3=CAN1 input timed out. Counts survive recovery. */
    uint32_t diagnostic_version;
    uint32_t input_age_ms, input_loss_count, last_input_loss_tick_ms;
    uint32_t last_input_loss_age_ms, input_max_gap_ms;
    uint8_t input_reason, last_input_loss_reason;
    uint32_t feedback_stale_count[4], feedback_max_gap_ms[5];
    uint32_t wheel_output_sign_flip_count[4];
    uint32_t zero_request_count, calibration_reset_count, control_max_gap_ms;
    uint32_t send_gate_drop_count;
} board_lower_diag_t;

extern volatile board_lower_diag_t lower_diag;
#endif

/* Fresh original input on UPPER / fresh chassis input on LOWER.
 * Telemetry only: no additional arming procedure or switch mapping. */
extern volatile uint8_t board_armed;
extern volatile uint32_t board_rx_count;
extern volatile uint32_t board_tx_fail_count;
extern volatile uint8_t board_remote_motor_mask;
extern volatile uint32_t board_chassis_rx_count;
extern volatile uint32_t board_yaw_rx_count;

void BOARD_LINK_TASK(void const *argument);
void board_link_poll(void);
uint8_t board_link_receive(uint32_t id, const uint8_t data[8]);
void board_note_input(void);
uint8_t board_motion_enabled(void);
uint8_t board_yaw_enabled(void);
uint8_t board_lower_online(void);
uint8_t board_prepare_chassis_input(void);
uint8_t board_take_chassis_reset(void);
void board_set_yaw_command(int16_t current);
void board_capture_feedback(uint8_t index, const uint8_t data[8]);
uint32_t board_motor_feedback_age(uint8_t index);
void board_send_motor(CAN_HandleTypeDef *can, uint32_t id, int16_t a,
                      int16_t b, int16_t c, int16_t d);
#endif
