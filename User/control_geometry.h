#ifndef CONTROL_GEOMETRY_H
#define CONTROL_GEOMETRY_H
#include <stdint.h>
#include <float.h>

typedef struct {
    float previous_wrapped;
    float continuous;
    uint8_t initialized;
} control_yaw_tracker_t;

static inline float control_abs(float x) { return x < 0.0f ? -x : x; }
static inline uint8_t control_finite(float x)
{
    return x == x && x <= FLT_MAX && x >= -FLT_MAX;
}
static inline float control_wrap_deg(float angle)
{
    int32_t turns;
    if (!control_finite(angle) || control_abs(angle) > 100000000.0f) return 0.0f;
    turns = (int32_t)(angle / 360.0f);
    angle -= (float)turns * 360.0f;
    if (angle >= 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}
static inline float control_unwrap_yaw(control_yaw_tracker_t *state, float wrapped)
{
    if (!control_finite(wrapped))
        return state->initialized && control_finite(state->continuous) ? state->continuous : 0.0f;
    if (!state->initialized || !control_finite(state->continuous) ||
        !control_finite(state->previous_wrapped))
    {
        state->previous_wrapped = wrapped; state->continuous = wrapped;
        state->initialized = 1;
    }
    else
    {
        state->continuous += control_wrap_deg(wrapped - state->previous_wrapped);
        state->previous_wrapped = wrapped;
    }
    return state->continuous;
}
static inline float control_encoder_yaw(uint16_t encoder, uint16_t zero, float sign)
{
    int32_t delta = (int32_t)encoder - (int32_t)zero;
    if (delta >= 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    return control_wrap_deg((float)delta * (360.0f / 8192.0f) * sign);
}
static inline float control_slew(float target, float current, float step)
{
    float delta = target - current;
    if (step <= 0.0f) return target;
    if (delta > step) delta = step;
    if (delta < -step) delta = -step;
    return current + delta;
}
static inline void control_slew_vector(float tx, float ty, float *x, float *y, float step)
{
    float dx = tx - *x, dy = ty - *y;
    float largest = control_abs(dx) > control_abs(dy) ? control_abs(dx) : control_abs(dy);
    if (step <= 0.0f || largest <= step) { *x = tx; *y = ty; }
    else { *x += dx * (step / largest); *y += dy * (step / largest); }
}
#endif
