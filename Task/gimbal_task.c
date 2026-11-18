#include "gimbal_task.h"
#include "control_config.h"
#include "control_geometry.h"
#include "yaw_control.h"
#if BOARD_ROLE == BOARD_UPPER
#include "board_link.h"
#endif

Sliding yaw;
Sliding pitch;

extern usb_t usb;
extern VTM_Data_t key;
gimbal_t gimbal;
#if BOARD_ROLE == BOARD_UPPER
static control_yaw_tracker_t yaw_tracker;
static control_yaw_guard_t yaw_guard;
static control_yaw_rate_observer_t yaw_rate_observer;
static uint8_t yaw_rc_moving, yaw_rc_tick_seen;
static uint32_t yaw_rc_last_tick;
volatile yaw_diag_t yaw_diag;

static void reset_yaw_rate_observer(void)
{
    control_yaw_rate_reset(&yaw_rate_observer);
    yaw_diag.angle_rate_dps = yaw_diag.gyro_mean_dps = 0;
    yaw_diag.rate_direction = YAW_RATE_DIRECTION_UNKNOWN;
    yaw_diag.last_angle_rate_dps = yaw_diag.last_gyro_mean_dps = 0;
    yaw_diag.last_rate_direction = YAW_RATE_DIRECTION_UNKNOWN;
}

static void reset_yaw_fault(void)
{
    control_yaw_guard_reset(&yaw_guard);
    yaw_rc_moving = yaw_rc_tick_seen = 0;
    yaw_diag.fault = YAW_FAULT_NONE;
    yaw_diag.fault_error_deg = yaw_diag.fault_rate_dps = yaw_diag.fault_raw_output = 0;
    yaw_diag.fault_angle_rate_dps = yaw_diag.fault_gyro_mean_dps = 0;
    yaw_diag.fault_rate_direction = YAW_RATE_DIRECTION_UNKNOWN;
    reset_yaw_rate_observer();
}

static float protected_yaw_output(float raw_output, float rate_dps)
{
    uint32_t previous_fault = yaw_guard.fault;
    yaw_diag.target_deg = gimbal.target_yaw;
    yaw_diag.measured_deg = gimbal.measured_yaw;
    yaw_diag.error_deg = gimbal.yaw_error;
    yaw_diag.rate_dps = rate_dps;
    yaw_diag.target_rate_dps = yaw.error.tar_differential;
    yaw_diag.raw_output = raw_output;
    yaw_diag.fault = control_yaw_guard_update(&yaw_guard, gimbal.yaw_error,
                                             rate_dps, raw_output, HAL_GetTick());
    if (!previous_fault && yaw_diag.fault)
    {
        yaw_diag.fault_error_deg = gimbal.yaw_error;
        yaw_diag.fault_rate_dps = rate_dps;
        yaw_diag.fault_raw_output = raw_output;
        yaw_diag.fault_angle_rate_dps = yaw_diag.angle_rate_dps;
        yaw_diag.fault_gyro_mean_dps = yaw_diag.gyro_mean_dps;
        yaw_diag.fault_rate_direction = yaw_diag.rate_direction;
    }
    /* Braking must not keep the previous torque sign for a slew-down ramp. */
    return yaw_diag.fault ? 0.0f : control_yaw_command(raw_output);
}

static void lock_yaw_target(float measured)
{
    SMC_Clear(&yaw);
    gimbal.target_yaw = measured;
    yaw.error.tar_now = measured;
    yaw.error.tar_last = measured;
}
#endif

static void km_control(gimbal_t *gimbal);
static void rc_control(gimbal_t *gimbal);
static void pitch_constraint(gimbal_t *gimbal);
static void yaw_constraint(gimbal_t *gimbal);

#define RC_DEADBAND 20
#define MOUSE_DEADBAND 3
#define RC_CH_CENTER 1024.0f
#define RC_CH_RANGE 660.0f
#define RC_PITCH_ABS_MAX_DEG 32.0f

#define PITCH_RATE_LPF_ALPHA 0.10f
#define PITCH_HOLD_ERR_DEG 0.20f
#define PITCH_HOLD_RATE_DPS 2.5f
#define PITCH_CMD_LIMIT 7000.0f
#define PITCH_CMD_DEADZONE 90.0f
#define PITCH_CMD_SLEW_STEP 120.0f

static float abs_f32(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float clamp_f32(float x, float min_v, float max_v)
{
    if (x > max_v)
    {
        return max_v;
    }
    if (x < min_v)
    {
        return min_v;
    }
    return x;
}

static float slew_limit_f32(float target, float current, float step)
{
    float delta = target - current;
    if (delta > step)
    {
        delta = step;
    }
    else if (delta < -step)
    {
        delta = -step;
    }
    return current + delta;
}

static int16_t rc_delta_with_deadband(uint16_t ch)
{
    int16_t delta = (int16_t)ch - 1024;
    if ((delta > -RC_DEADBAND) && (delta < RC_DEADBAND))
    {
        return 0;
    }
    return delta;
}

static int16_t mouse_delta_with_deadband(int16_t v)
{
    if ((v > -MOUSE_DEADBAND) && (v < MOUSE_DEADBAND))
    {
        return 0;
    }
    return v;
}

void GIMBAL_TASK(void const *pvParameters)
{
    uint32_t xLastWakeTime = osKernelSysTick();
    const portTickType xFrequency = 1;

    SMC_Init(&pitch);
    SMC_Init(&yaw);
#if BOARD_ROLE == BOARD_UPPER
    reset_yaw_fault();
#endif

    gimbal.target_yaw = 0.0f;
    gimbal.target_pitch = 0.0f;

#if BOARD_ROLE == BOARD_UPPER
    SMC_SetParam_Exponent(&yaw, YAW_SMC_J, YAW_SMC_K, YAW_SMC_C, YAW_SMC_EPSILON, 2.0f, 16384.0f, 0.0f);
#else
    SMC_SetParam_Exponent(&yaw, 0.8f, 120.0f, 15.0f, 0.8f, 2.0f, 16384.0f, 0.0f);
#endif
	SMC_SetParam_EISMC(&pitch, 1.0f, 90.0f, 10.0f, 2.5f, 12.0f, 1.0f, 12000.0f, 0.0f);

	osDelay(500);
#if BOARD_ROLE == BOARD_UPPER
    /* Do not replay 500 missed 1ms iterations after the startup delay. */
    xLastWakeTime = osKernelSysTick();
#endif

    /* 启动时将目标角锁到当前姿态，避免追 0 引起抖动 */
#if BOARD_ROLE == BOARD_UPPER
    gimbal.measured_yaw = control_unwrap_yaw(&yaw_tracker, mahony.YAW);
    lock_yaw_target(gimbal.measured_yaw);
#else
    gimbal.target_yaw = mahony.YAW;
#endif
    gimbal.target_pitch = mahony.PITCH;
    pc_note_attitude(mahony.YAW);
    SMC_IntegralClear(&yaw);
    SMC_IntegralClear(&pitch);

    float pitch_rate_dps_lpf = 0.0f;
    float yaw_rate_dps_lpf = 0.0f;
    float pitch_cmd_last = 0.0f;

    uint8_t last_sw = key.sw;
#if BOARD_ROLE == BOARD_UPPER
    uint8_t yaw_was_enabled = 0;
#endif
    while (1)
    {
            pc_note_attitude(mahony.YAW);
#if BOARD_ROLE == BOARD_SINGLE
            if (pc_stop_active())
            {
                pc_clear_command();
                CAN_cmd_pitch(0); CAN_cmd_yaw(0);
                SMC_Clear(&yaw); SMC_Clear(&pitch);
                gimbal.target_yaw = mahony.YAW;
                gimbal.target_pitch = mahony.PITCH;
                pitch_cmd_last = yaw_rate_dps_lpf = pitch_rate_dps_lpf = 0;
                osDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
#endif
#if BOARD_ROLE == BOARD_UPPER
            uint8_t valid_yaw = control_finite(mahony.YAW) && control_finite(mahony.g_smc[2]);
            uint8_t valid_pitch = control_finite(mahony.PITCH) && control_finite(mahony.g_smc[1]);
            if (calibrate.flag == 1 || !board_motion_enabled())
            {
                pc_clear_command();
                CAN_cmd_pitch(0); CAN_cmd_yaw(0);
                if (calibrate.flag == 1) CAN_cmd_shoot(0, 0, 0);
                if (valid_yaw)
                {
                    yaw_tracker.initialized = 0;
                    gimbal.measured_yaw = control_unwrap_yaw(&yaw_tracker, mahony.YAW);
                    lock_yaw_target(gimbal.measured_yaw);
                }
                if (valid_pitch)
                {
                    gimbal.target_pitch = mahony.PITCH;
                    SMC_IntegralClear(&pitch);
                }
                pitch_cmd_last = 0;
                control_yaw_guard_pause(&yaw_guard);
                yaw_rc_moving = yaw_rc_tick_seen = 0;
                reset_yaw_rate_observer();
                yaw_diag.output = 0;
                yaw_rate_dps_lpf = 0; pitch_rate_dps_lpf = 0;
                gimbal.yaw_output = 0; gimbal.pitch_output = 0;
                yaw_was_enabled = 0;
                osDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            if (valid_yaw)
            {
                gimbal.measured_yaw = control_unwrap_yaw(&yaw_tracker, mahony.YAW);
                if (!yaw_was_enabled)
                {
                    lock_yaw_target(gimbal.measured_yaw);
                    yaw_was_enabled = 1;
                }
            }
            else
            {
                /* Invalid yaw feedback must not disable a healthy pitch axis. */
                SMC_Clear(&yaw);
                yaw_tracker.initialized = 0;
                control_yaw_guard_pause(&yaw_guard);
                yaw_rc_moving = yaw_rc_tick_seen = 0;
                reset_yaw_rate_observer();
                yaw_rate_dps_lpf = 0;
                yaw_was_enabled = 0;
            }
#endif
            if (key.sw != last_sw)
            {
                last_sw = key.sw;
#if BOARD_ROLE == BOARD_UPPER
                if (valid_yaw) lock_yaw_target(gimbal.measured_yaw);
                reset_yaw_fault();
#else
                gimbal.target_yaw = mahony.YAW;
#endif
                gimbal.target_pitch = mahony.PITCH;
                SMC_IntegralClear(&yaw);
                SMC_IntegralClear(&pitch);
            }

			if (key.sw == 1)
			{
				km_control(&gimbal);
			}
			else if (key.sw == 0)
			{
                pc_clear_command();
				rc_control(&gimbal);
			}

            else pc_clear_command();
				
#if BOARD_ROLE != BOARD_UPPER
      zerocheckangle(&gimbal.target_yaw, &mahony.YAW);
#endif
			pitch_constraint(&gimbal);
			yaw_constraint(&gimbal);

        float pitch_rate_dps = mahony.g_smc[1] * 57.2958f;
        float yaw_rate_dps = mahony.g_smc[2] * 57.2958f;
#if BOARD_ROLE == BOARD_UPPER
        if (valid_yaw)
        {
        yaw_diag.gyro_z_dps = yaw_rate_dps;
        yaw_rate_dps *= YAW_RATE_SIGN;
        control_yaw_rate_observe(&yaw_rate_observer, gimbal.measured_yaw, yaw_rate_dps, HAL_GetTick());
        yaw_diag.angle_rate_dps = yaw_rate_observer.angle_rate_dps;
        yaw_diag.gyro_mean_dps = yaw_rate_observer.gyro_mean_dps;
        yaw_diag.rate_direction = yaw_rate_observer.direction;
        yaw_diag.last_angle_rate_dps = yaw_rate_observer.last_angle_rate_dps;
        yaw_diag.last_gyro_mean_dps = yaw_rate_observer.last_gyro_mean_dps;
        yaw_diag.last_rate_direction = yaw_rate_observer.last_direction;
#endif
        yaw_rate_dps_lpf += PITCH_RATE_LPF_ALPHA * (yaw_rate_dps - yaw_rate_dps_lpf);

#if BOARD_ROLE == BOARD_UPPER
        /* A +/-360 representation change is not a commanded angular velocity. */
        yaw.error.tar_last = gimbal.target_yaw - control_wrap_deg(gimbal.target_yaw - yaw.error.tar_last);
        SMC_ErrorUpdate(&yaw, gimbal.target_yaw, gimbal.measured_yaw, yaw_rate_dps_lpf);
#else
        SMC_ErrorUpdate(&yaw, gimbal.target_yaw, mahony.YAW, yaw_rate_dps_lpf);
#endif
        gimbal.yaw_output = SMC_Calculate(&yaw);
#if BOARD_ROLE == BOARD_UPPER
        gimbal.yaw_error = control_wrap_deg(gimbal.target_yaw - gimbal.measured_yaw);
        gimbal.yaw_output = protected_yaw_output(gimbal.yaw_output, yaw_rate_dps_lpf);
        if (control_abs(gimbal.yaw_error) < YAW_HOLD_ERROR_DEG &&
            control_abs(yaw_rate_dps_lpf) < YAW_HOLD_RATE_DPS)
        {
            gimbal.yaw_output = 0;
            SMC_IntegralClear(&yaw);
        }
        yaw_diag.output = gimbal.yaw_output;
        }
        else
        {
            gimbal.yaw_output = yaw_diag.output = 0;
        }
        if (valid_pitch && control_finite(gimbal.target_pitch))
        {
#endif
        pitch_rate_dps_lpf += PITCH_RATE_LPF_ALPHA * (pitch_rate_dps - pitch_rate_dps_lpf);
			SMC_ErrorUpdate(&pitch, gimbal.target_pitch, mahony.PITCH, pitch_rate_dps_lpf);
			gimbal.pitch_output = SMC_Calculate(&pitch);

        gimbal.pitch_output = -1.0f * gimbal.pitch_output;

        if ((abs_f32(gimbal.target_pitch - mahony.PITCH) < PITCH_HOLD_ERR_DEG)
            && (abs_f32(pitch_rate_dps_lpf) < PITCH_HOLD_RATE_DPS))
        {
            gimbal.pitch_output = 0.0f;
            SMC_IntegralClear(&pitch);
        }

        gimbal.pitch_output = clamp_f32(gimbal.pitch_output, -PITCH_CMD_LIMIT, PITCH_CMD_LIMIT);
        gimbal.pitch_output = slew_limit_f32(gimbal.pitch_output, pitch_cmd_last, PITCH_CMD_SLEW_STEP);
        pitch_cmd_last = gimbal.pitch_output;

        if (abs_f32(gimbal.pitch_output) < PITCH_CMD_DEADZONE)
        {
            gimbal.pitch_output = 0.0f;
        }
#if BOARD_ROLE == BOARD_UPPER
        }
        else
        {
            SMC_Clear(&pitch);
            pitch_rate_dps_lpf = pitch_cmd_last = 0;
            gimbal.pitch_output = 0;
        }
#endif
			
			if (calibrate.flag != 1)
			{
        CAN_cmd_pitch((int16_t)gimbal.pitch_output);
        CAN_cmd_yaw((int16_t)gimbal.yaw_output);
			}
			else if (calibrate.flag == 1)
			{
				CAN_cmd_pitch(0);
				CAN_cmd_shoot(0,0,0);
                gimbal.target_yaw = mahony.YAW;
                gimbal.target_pitch = mahony.PITCH;
                SMC_IntegralClear(&yaw);
                SMC_IntegralClear(&pitch);
                pitch_cmd_last = 0.0f;
			}
        osDelayUntil(&xLastWakeTime,xFrequency);
    }
}

static void km_control(gimbal_t *gimbal)
{
    float aim_yaw, aim_pitch;
    uint8_t aim_ready = pc_take_aim(&aim_yaw, &aim_pitch);
    if (key.mouse_btn_r == 0)
    {
        int16_t mx = mouse_delta_with_deadband(key.mouse_x);
        int16_t my = mouse_delta_with_deadband(key.mouse_y);
        gimbal->target_yaw += MOUSE_X_MAP * mx * 0.001f;
        gimbal->target_pitch += MOUSE_Y_MAP * my * 0.001f;
    }
    else if (key.mouse_btn_r == 1)
    {
        if (aim_ready) {
            gimbal->target_yaw += aim_yaw;
            gimbal->target_pitch += aim_pitch;
        }
    }
}

static void rc_control(gimbal_t *gimbal)
{
    int16_t ch3_delta = rc_delta_with_deadband(key.ch3);
    float pitch_axis = ((float)key.ch2 - RC_CH_CENTER) / RC_CH_RANGE;
#if BOARD_ROLE == BOARD_UPPER
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = yaw_rc_tick_seen ? (uint32_t)(now - yaw_rc_last_tick) : 1u;
    yaw_rc_last_tick = now; yaw_rc_tick_seen = 1;
    /* Replaying task iterations or resuming a debugger must not accumulate
     * a burst of stick motion. Normal control uses the measured 1ms period. */
    if (elapsed > YAW_RC_MAX_STEP_MS) elapsed = 1u;
    if (ch3_delta != 0)
    {
        float axis = clamp_f32((float)ch3_delta / RC_CH_RANGE, -1.0f, 1.0f);
        float lead, requested, constrained;
        if (!yaw_rc_moving)
        {
            lock_yaw_target(gimbal->measured_yaw);
            yaw_rc_moving = 1;
        }
        requested = gimbal->target_yaw + MOUSE_X_MAP * axis * YAW_RC_MAX_RATE_DPS * ((float)elapsed * 0.001f);
        lead = control_wrap_deg(requested - gimbal->measured_yaw);
        constrained = gimbal->measured_yaw +
            clamp_f32(lead, -YAW_RC_MAX_LEAD_DEG, YAW_RC_MAX_LEAD_DEG);
        /* A lead-limit reference correction is not a velocity command. */
        yaw.error.tar_last += constrained - requested;
        gimbal->target_yaw = constrained;
    }
    else if (yaw_rc_moving)
    {
        /* Latch once on stick release; subsequent centered iterations keep
         * this reference so a disturbance produces restoring torque. */
        lock_yaw_target(gimbal->measured_yaw);
        yaw_rc_moving = 0;
    }
#else
    gimbal->target_yaw += MOUSE_X_MAP * ch3_delta * 0.0001f;
#endif
    gimbal->target_pitch = clamp_f32(MOUSE_Y_MAP * pitch_axis * RC_PITCH_ABS_MAX_DEG,
                                     -RC_PITCH_ABS_MAX_DEG,
                                     RC_PITCH_ABS_MAX_DEG);
}

static void pitch_constraint(gimbal_t *gimbal)
{
    gimbal->target_pitch = (gimbal->target_pitch > 32.0f) ? 32.0f : 
                           (gimbal->target_pitch < -32.0f) ? -32.0f : 
                           gimbal->target_pitch;
}

static void yaw_constraint(gimbal_t *gimbal)
{
#if BOARD_ROLE == BOARD_UPPER
    if (!control_finite(gimbal->target_yaw)) lock_yaw_target(gimbal->measured_yaw);
    gimbal->target_yaw = gimbal->measured_yaw +
                        control_wrap_deg(gimbal->target_yaw - gimbal->measured_yaw);
#else
    gimbal->target_yaw = (gimbal->target_yaw > 180.0f) ? 180.0f : 
                         (gimbal->target_yaw < -180.0f) ? -180.0f : 
                         gimbal->target_yaw;
#endif
}
