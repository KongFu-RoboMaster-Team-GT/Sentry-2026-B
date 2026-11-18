#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H
#include "board_config.h"

/* Measured with gun aligned to chassis front. Encoder ID/position order:
 * LF=1, LR=2, RR=3, RF=4. Positive relative angle is CCW viewed from above. */
#define CHASSIS_YAW_ZERO_ECD 3156u
#define CHASSIS_YAW_ECD_SIGN 1.0f

/* Receiver decoding is unchanged. Measured forward push: ch1 increases
 * (1028 -> 1807); ch0 stays within its original neutral deadband. */
#define CHASSIS_RC_FORWARD_CHANNEL 1u
#define CHASSIS_RC_FORWARD_SIGN 1.0f
#define CHASSIS_RC_LEFT_CHANNEL 0u
#define CHASSIS_RC_LEFT_SIGN (-1.0f)

/* Preserve the measured full-forward speed, remove the center discontinuity.
 * Symmetric endpoints are provisional; raw receiver decoding is unchanged. */
#define CHASSIS_RC_CENTER_DEADBAND 20.0f
#define CHASSIS_RC_FULL_SCALE 783.0f
#define CHASSIS_RC_MAX_SPEED_MPS (783.0f / 660.0f)
#define CHASSIS_STOP_MOTOR_RPS 1.0f
/* Bound zero-reference braking; stay released until a new motion request. */
#define CHASSIS_BRAKE_TIMEOUT_MS 300u

#define CHASSIS_LINEAR_ACCEL_MPS2 1.5f
#define CHASSIS_ANGULAR_ACCEL_RPS2 8.0f
/* Same original spin speed/direction: rad/s, one chassis turn per second.
 * Original RC ch7/key.pause is the enable; translation remains gun-relative. */
#define CHASSIS_SPIN_OMEGA_RPS 6.28318530718f

/* Synchronous observer reported OPPOSITE with -1: restore angle/rate agreement.
 * This changes velocity feedback only, not the physical motor output sign. */
#define YAW_RATE_SIGN 1.0f
#define YAW_RC_MAX_RATE_DPS 120.0f
#define YAW_RC_MAX_LEAD_DEG 30.0f
#define YAW_RC_MAX_STEP_MS 5u
/* UPPER response tuning: double the previous position stiffness (c=4 -> 8).
 * At a fixed target: position gain = J*K*c, rate gain = J*(K+c).
 * These are provisional gains, not a verified plant tuning. */
#define YAW_SMC_J 0.8f
#define YAW_SMC_K 120.0f
#define YAW_SMC_C 8.0f
#define YAW_SMC_EPSILON 0.8f
/* Command-unit limit, not a measured current in amperes. */
#define YAW_OUTPUT_SIGN 1.0f
#define YAW_OUTPUT_LIMIT 16384.0f
#define YAW_HOLD_ERROR_DEG 0.15f
#define YAW_HOLD_RATE_DPS 1.5f
/* 0: disable the commissioning overspeed/tracking-error stop latch.
 * Non-finite commands and stale input are still rejected independently. */
#ifndef YAW_MOTION_GUARD_ENABLE
#define YAW_MOTION_GUARD_ENABLE 0
#endif
#define YAW_FAULT_RATE_DPS 180.0f
#define YAW_FAULT_ERROR_DEG 45.0f
#define YAW_FAULT_DELAY_MS 50u
#define YAW_RATE_OBSERVER_WINDOW_MS 20u
#define YAW_RATE_OBSERVER_MAX_GAP_MS 10u
#define YAW_RATE_OBSERVER_MIN_DPS 10.0f

#endif
