#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define BOARD_SINGLE 0
#define BOARD_UPPER 1
#define BOARD_LOWER 2
/* FINAL-2026 keeps the original single-board path. */
#ifndef BOARD_ROLE
#define BOARD_ROLE BOARD_SINGLE
#endif
#if BOARD_ROLE < BOARD_SINGLE || BOARD_ROLE > BOARD_LOWER
#error Invalid BOARD_ROLE
#endif

#define BOARD_LINK_TIMEOUT_MS 100u
#define BOARD_RC_TIMEOUT_MS 100u
#define BOARD_MOTOR_TIMEOUT_MS 100u
#define BOARD_PROTOCOL_VERSION 3u
#define BOARD_CHASSIS_INPUT_PERIOD_MS 10u
#define BOARD_YAW_PERIOD_MS 2u
#define BOARD_STATUS_PERIOD_MS 20u
#define BOARD_YAW_CACHE_TIMEOUT_MS 20u
/* UPPER CAN1 pending-command lifetime, serviced every 1ms and before enqueue.
 * Abort is asynchronous: do not reuse/send until cancellation completes. */
#define BOARD_CAN1_TX_LIFETIME_MS 10u

#endif
