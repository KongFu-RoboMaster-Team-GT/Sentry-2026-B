#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "alg_pid.h"
#include "control_geometry.h"
#include "control_config.h"

/* LOWER-only callers: keep the shared shooter/legacy PID algorithm intact.
 * Reject only integration which drives an already saturated output farther
 * into saturation. Integration which unwinds saturation remains enabled. */
static inline void chassis_pid_calculate(PID_t *pid, float *output)
{
    float old_integral = pid->Integral_Error;
    float old_error = pid->Pre_Error;
    float old_target = pid->Pre_Target;
    float raw, delta_i;
    PID_Calculate(pid, output);
    raw = pid->K_P * pid->Pre_Error + pid->K_I * pid->Integral_Error +
          pid->K_D * (pid->Pre_Error - old_error) / pid->D_T +
          pid->K_F * (pid->Target - old_target);
    delta_i = pid->K_I * (pid->Integral_Error - old_integral);
    if (pid->Out_Max > 0 &&
        ((raw > pid->Out_Max && delta_i > 0) ||
         (raw < -pid->Out_Max && delta_i < 0)))
    {
        pid->Integral_Error = old_integral;
        raw -= delta_i;
        if (raw > pid->Out_Max) raw = pid->Out_Max;
        if (raw < -pid->Out_Max) raw = -pid->Out_Max;
        *output = pid->Pre_Out = raw;
    }
}

static inline void chassis_speed_pid(PID_t *pid, float *output)
{
    /* A wheel reversal/zero crossing must not carry the old drive bias.
     * This is per wheel: the other wheels can still be moving. */
    if (pid->Target != pid->Pre_Target && pid->Target * pid->Pre_Target <= 0)
        pid->Integral_Error = 0;
    chassis_pid_calculate(pid, output);
}

static inline float chassis_follow_control(PID_t *pid, float theta, uint8_t active)
{
    float output;
    if (!active || control_abs(theta) <= pid->Dead_Zone)
    {
        PID_Clear_Error(pid);
        pid->Target = pid->Now = 0;
        return 0;
    }
    if (theta * pid->Pre_Target < 0) pid->Integral_Error = 0;
    PID_Set_Values(pid, theta, 0);
    chassis_pid_calculate(pid, &output);
    return output;
}

static inline float chassis_rc_axis(uint16_t channel)
{
    float delta = (float)channel - 1024.0f;
    float magnitude = control_abs(delta);
    if (magnitude <= CHASSIS_RC_CENTER_DEADBAND) return 0;
    if (magnitude > CHASSIS_RC_FULL_SCALE) magnitude = CHASSIS_RC_FULL_SCALE;
    magnitude = (magnitude - CHASSIS_RC_CENTER_DEADBAND) /
                (CHASSIS_RC_FULL_SCALE - CHASSIS_RC_CENTER_DEADBAND) * CHASSIS_RC_MAX_SPEED_MPS;
    return delta < 0 ? -magnitude : magnitude;
}

#endif
