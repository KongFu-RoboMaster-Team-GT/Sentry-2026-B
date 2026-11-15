#ifndef YAW_CONTROL_H
#define YAW_CONTROL_H
#include "control_config.h"
#include "control_geometry.h"

enum {
    YAW_FAULT_NONE = 0,
    YAW_FAULT_OVERSPEED = 1,
    YAW_FAULT_TRACKING = 2,
    YAW_FAULT_INVALID = 3
};

typedef struct {
    uint32_t fault;
    uint32_t rate_since, error_since;
    uint8_t rate_active, error_active;
} control_yaw_guard_t;

enum {
    YAW_RATE_DIRECTION_UNKNOWN = 0,
    YAW_RATE_DIRECTION_SAME = 1,
    YAW_RATE_DIRECTION_OPPOSITE = 2
};

typedef struct {
    uint32_t start_tick, last_tick;
    float start_yaw, previous_rate, integrated_rate;
    float angle_rate_dps, gyro_mean_dps;
    uint32_t direction;
    float last_angle_rate_dps, last_gyro_mean_dps;
    uint32_t last_direction;
    uint8_t initialized;
} control_yaw_rate_observer_t;

typedef struct {
    uint32_t fault;
    float target_deg, measured_deg, error_deg;
    float gyro_z_dps, rate_dps, target_rate_dps;
    float raw_output, output;
    float fault_error_deg, fault_rate_dps, fault_raw_output;
    float angle_rate_dps, gyro_mean_dps;
    uint32_t rate_direction;
    float last_angle_rate_dps, last_gyro_mean_dps;
    uint32_t last_rate_direction;
    float fault_angle_rate_dps, fault_gyro_mean_dps;
    uint32_t fault_rate_direction;
} yaw_diag_t;

/* Published by UPPER; fault snapshot persists until a mode change or reboot. */
extern volatile yaw_diag_t yaw_diag;

static inline void control_yaw_rate_reset(control_yaw_rate_observer_t *state)
{
    state->initialized = 0;
    state->angle_rate_dps = state->gyro_mean_dps = 0;
    state->direction = YAW_RATE_DIRECTION_UNKNOWN;
    state->last_angle_rate_dps = state->last_gyro_mean_dps = 0;
    state->last_direction = YAW_RATE_DIRECTION_UNKNOWN;
}

/* Observation only: compare angle displacement and signed raw gyro over the
 * same window. No control sign is selected or changed by this measurement. */
static inline void control_yaw_rate_observe(control_yaw_rate_observer_t *state,
                                           float continuous_yaw, float rate_dps,
                                           uint32_t now)
{
    uint32_t dt, elapsed;
    float scale;
    if (!control_finite(continuous_yaw) || !control_finite(rate_dps))
    { control_yaw_rate_reset(state); return; }
    if (!state->initialized || (uint32_t)(now - state->last_tick) > YAW_RATE_OBSERVER_MAX_GAP_MS)
    {
        control_yaw_rate_reset(state);
        state->start_tick = state->last_tick = now;
        state->start_yaw = continuous_yaw; state->previous_rate = rate_dps;
        state->integrated_rate = 0; state->initialized = 1;
        return;
    }
    dt = (uint32_t)(now - state->last_tick);
    if (!dt) return;
    state->integrated_rate += 0.5f * (state->previous_rate + rate_dps) * ((float)dt * 0.001f);
    state->previous_rate = rate_dps; state->last_tick = now;
    elapsed = (uint32_t)(now - state->start_tick);
    if (elapsed < YAW_RATE_OBSERVER_WINDOW_MS) return;
    scale = 1000.0f / (float)elapsed;
    state->angle_rate_dps = (continuous_yaw - state->start_yaw) * scale;
    state->gyro_mean_dps = state->integrated_rate * scale;
    state->direction = YAW_RATE_DIRECTION_UNKNOWN;
    if (control_abs(state->angle_rate_dps) >= YAW_RATE_OBSERVER_MIN_DPS &&
        control_abs(state->gyro_mean_dps) >= YAW_RATE_OBSERVER_MIN_DPS)
    {
        state->direction = ((state->angle_rate_dps > 0) == (state->gyro_mean_dps > 0)) ?
                            YAW_RATE_DIRECTION_SAME : YAW_RATE_DIRECTION_OPPOSITE;
        state->last_direction = state->direction;
        state->last_angle_rate_dps = state->angle_rate_dps;
        state->last_gyro_mean_dps = state->gyro_mean_dps;
    }
    state->start_tick = now; state->start_yaw = continuous_yaw; state->integrated_rate = 0;
}

static inline void control_yaw_guard_pause(control_yaw_guard_t *state)
{
    state->rate_active = state->error_active = 0;
}

static inline void control_yaw_guard_reset(control_yaw_guard_t *state)
{
    state->fault = YAW_FAULT_NONE;
    state->rate_since = state->error_since = 0;
    control_yaw_guard_pause(state);
}

static inline uint32_t control_yaw_guard_update(control_yaw_guard_t *state,
                                               float error, float rate,
                                               float output, uint32_t now)
{
#if !YAW_MOTION_GUARD_ENABLE
    /* A disabled motion guard cannot retain an old overspeed/tracking stop. */
    if (state->fault == YAW_FAULT_OVERSPEED || state->fault == YAW_FAULT_TRACKING)
        state->fault = YAW_FAULT_NONE;
    control_yaw_guard_pause(state);
#endif
    if (state->fault) return state->fault;
    if (!control_finite(error) || !control_finite(rate) || !control_finite(output))
        return state->fault = YAW_FAULT_INVALID;

#if YAW_MOTION_GUARD_ENABLE
    if (control_abs(rate) > YAW_FAULT_RATE_DPS)
    {
        if (!state->rate_active) { state->rate_since = now; state->rate_active = 1; }
        if ((uint32_t)(now - state->rate_since) >= YAW_FAULT_DELAY_MS)
            return state->fault = YAW_FAULT_OVERSPEED;
    }
    else state->rate_active = 0;

    if (control_abs(error) > YAW_FAULT_ERROR_DEG)
    {
        if (!state->error_active) { state->error_since = now; state->error_active = 1; }
        if ((uint32_t)(now - state->error_since) >= YAW_FAULT_DELAY_MS)
            return state->fault = YAW_FAULT_TRACKING;
    }
    else state->error_active = 0;
#else
    (void)now;
#endif
    return YAW_FAULT_NONE;
}

static inline float control_yaw_command(float raw_output)
{
    float command;
    if (!control_finite(raw_output)) return 0;
    command = YAW_OUTPUT_SIGN * raw_output;
    if (command > YAW_OUTPUT_LIMIT) return YAW_OUTPUT_LIMIT;
    if (command < -YAW_OUTPUT_LIMIT) return -YAW_OUTPUT_LIMIT;
    return command;
}
#endif
