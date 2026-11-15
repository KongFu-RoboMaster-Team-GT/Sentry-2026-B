#include "chassis_task.h"
#include "motion.h"
#include "control_geometry.h"
#if BOARD_ROLE == BOARD_LOWER
#include "board_link.h"
#include "chassis_control.h"
#endif

PID_t chassis[4];
PID_t follow_pid;
chassis_motion_value_t chassis_motion;
extern VTM_Data_t key;

static uint8_t chassis_follow_enabled = 0;
static uint8_t last_key_f = 0;
#if BOARD_ROLE == BOARD_LOWER
static float gimbal_vx_applied, gimbal_vy_applied, chassis_omega_applied;
static uint8_t chassis_idle_latched = 1, chassis_brake_timing;
static uint32_t chassis_brake_start;
static uint8_t chassis_last_stop_requested = 1;
static int16_t chassis_last_command[4];
#endif

static void theta_calculate(chassis_motion_value_t *chassis_motion);
static void output_map(float *value);
static void km_control(void);
static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion);
static void gimbal_coordinate_resolution(float theta, chassis_motion_value_t *chassis_motion);
static void rc_control(void);

#define CHASSIS_RC_DEADBAND 20
#define CHASSIS_ZERO_EPS_V  0.02f
#define CHASSIS_ZERO_EPS_W  0.05f

/* 底盘跟随 PID 参数（theta 单位：度，输出单位：rad/s）
 * 如果跟随方向反了，将 FOLLOW_KP 改为负值 */
#define FOLLOW_KP        0.05f
#define FOLLOW_KI        0.002f
#define FOLLOW_OUT_MAX   6.0f
#define FOLLOW_DEAD_ZONE 2.0f

#if BOARD_ROLE == BOARD_LOWER
static uint8_t chassis_can_release(uint8_t stop_requested)
{
    uint8_t i;
    uint8_t slow = 1;
    if (!stop_requested)
    {
        chassis_idle_latched = chassis_brake_timing = 0;
        return 0;
    }
    /* Feedback noise or an external push must not restart a stopped chassis. */
    if (chassis_idle_latched) return 1;
    if (gimbal_vx_applied != 0 || gimbal_vy_applied != 0 ||
        chassis_omega_applied != 0) return 0;
    if (!chassis_brake_timing)
    {
        chassis_brake_start = HAL_GetTick();
        chassis_brake_timing = 1;
    }
    for (i = 0; i < 4; ++i)
        if (fabsf(chassis_motion.wheel_now_omega[i]) >= CHASSIS_STOP_MOTOR_RPS) slow = 0;
    if (slow || (uint32_t)(HAL_GetTick() - chassis_brake_start) >= CHASSIS_BRAKE_TIMEOUT_MS)
    {
        if (!slow) ++lower_diag.brake_timeout_count;
        chassis_idle_latched = 1;
        chassis_brake_timing = 0;
    }
    return chassis_idle_latched;
}
#endif

#if BOARD_ROLE != BOARD_LOWER
static int16_t rc_delta_with_deadband(uint16_t ch)
{
	int16_t delta = (int16_t)ch - 1024;
	if ((delta > -CHASSIS_RC_DEADBAND) && (delta < CHASSIS_RC_DEADBAND))
	{
		return 0;
	}
	return delta;
}
#endif

void CHASSIS_TASK(void const * argument)
{
    uint32_t xLastWakeTime = osKernelSysTick();
    const portTickType xFrequency = 1;
#if BOARD_ROLE == BOARD_LOWER
    uint32_t previous_loop_tick = HAL_GetTick();
    lower_diag.diagnostic_version = 1;
#endif
    PID_Clear_Error(&chassis[0]);
    PID_Clear_Error(&chassis[1]);
    PID_Clear_Error(&chassis[2]);
    PID_Clear_Error(&chassis[3]);

    PID_Init(&chassis[0], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[1], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[2], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&chassis[3], 0.95f, 0.1f, 0.0f, 0.0f, 10.0f, 10.0f, 0.0f);
    PID_Init(&follow_pid, FOLLOW_KP, FOLLOW_KI, 0.0f, 0.0f, 1.0f, FOLLOW_OUT_MAX, FOLLOW_DEAD_ZONE);

	  chassis_motion.gimbal_target_vx = 0.0f;
		chassis_motion.gimbal_target_vy = 0.0f;
		chassis_motion.chassis_target_omega = 0.0f;
    while(1)
    {
#if BOARD_ROLE == BOARD_SINGLE
            if (pc_stop_active()) {
                uint8_t i;
                for (i = 0; i < 4; ++i) PID_Clear_Error(&chassis[i]);
                PID_Clear_Error(&follow_pid);
                CAN_cmd_chassis(0, 0, 0, 0);
                osDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
#endif
#if BOARD_ROLE == BOARD_LOWER
            uint8_t motor_index;
            uint8_t previous_stale = lower_diag.stale_wheel_mask;
            uint32_t loop_tick = HAL_GetTick();
            uint32_t loop_gap = (uint32_t)(loop_tick - previous_loop_tick);
            previous_loop_tick = loop_tick;
            if (loop_gap > lower_diag.control_max_gap_ms) lower_diag.control_max_gap_ms = loop_gap;
            ++lower_diag.chassis_loop_count;
            lower_diag.stale_wheel_mask = 0;
            for (motor_index = 0; motor_index < 5; ++motor_index)
            {
                lower_diag.feedback_age_ms[motor_index] = board_motor_feedback_age(motor_index);
                if (motor_index < 4 && lower_diag.feedback_age_ms[motor_index] > BOARD_MOTOR_TIMEOUT_MS)
                {
                    lower_diag.stale_wheel_mask |= 1u << motor_index;
                    if (!(previous_stale & (1u << motor_index)))
                        ++lower_diag.feedback_stale_count[motor_index];
                }
            }
            lower_diag.braking = 0;
            /* Only the input source changes. The original speed loop below
             * still executes every 1ms using LOWER's local CAN2 feedback. */
            if (!board_prepare_chassis_input())
            {
                uint8_t i;
                for (i = 0; i < 4; ++i) PID_Clear_Error(&chassis[i]);
                PID_Clear_Error(&follow_pid);
                gimbal_vx_applied = gimbal_vy_applied = chassis_omega_applied = 0.0f;
                chassis_idle_latched = lower_diag.idle_latched = 1;
                chassis_brake_timing = 0;
                for (i = 0; i < 4; ++i)
                {
                    chassis_motion.wheel_target_omega[i] = 0;
                    chassis_motion.motor_output_value[i] = 0;
                    chassis_last_command[i] = 0;
                }
                CAN_cmd_chassis(0, 0, 0, 0);
                osDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
            if (board_take_chassis_reset())
            {
                uint8_t i;
                ++lower_diag.calibration_reset_count;
                for (i = 0; i < 4; ++i) PID_Clear_Error(&chassis[i]);
            }
#endif
					chassis_motion.now_ecd = get_yaw_gimbal_motor_measure_point()->ecd;
					theta_calculate(&chassis_motion);
					
					for (int i = 0; i < 4; i++)
					{
							chassis_motion.wheel_now_omega[i] = get_chassis_motor_measure_point(i)->speed_rpm * 0.1047f;
					}
					if (key.sw == 1)
					{
						km_control();
					}
					else if (key.sw == 0)
					{
						rc_control();
					}

					/* 底盘跟随：当跟随使能且未处于小陀螺模式时，用 theta PID 控制底盘旋转 */
					uint8_t spinning = (key.sw == 1) ? key.key_shift : (key.pause != 0u);
#if BOARD_ROLE == BOARD_LOWER
                    {
                        float follow_omega = chassis_follow_control(&follow_pid, chassis_motion.theta,
                                                                    chassis_follow_enabled && !spinning);
                        if (chassis_follow_enabled && !spinning)
                            chassis_motion.chassis_target_omega = follow_omega;
                    }
#else
					if (chassis_follow_enabled && !spinning)
					{
						float follow_omega;
						PID_Set_Values(&follow_pid, chassis_motion.theta, 0.0f);
						PID_Calculate(&follow_pid, &follow_omega);
						chassis_motion.chassis_target_omega = follow_omega;
					}
#endif

#if BOARD_ROLE == BOARD_LOWER
                    uint8_t stop_requested = chassis_motion.gimbal_target_vx == 0.0f &&
                        chassis_motion.gimbal_target_vy == 0.0f &&
                        chassis_motion.chassis_target_omega == 0.0f;
                    if (stop_requested && !chassis_last_stop_requested) ++lower_diag.zero_request_count;
                    chassis_last_stop_requested = stop_requested;
#endif
					gimbal_coordinate_resolution(chassis_motion.theta, &chassis_motion);
#if BOARD_ROLE == BOARD_LOWER
                    uint8_t release_chassis = chassis_can_release(stop_requested);
                    lower_diag.idle_latched = chassis_idle_latched;
#endif
			if (
#if BOARD_ROLE == BOARD_LOWER
                release_chassis
#else
                (fabsf(chassis_motion.chassis_target_vx) < CHASSIS_ZERO_EPS_V)
			    && (fabsf(chassis_motion.chassis_target_vy) < CHASSIS_ZERO_EPS_V)
			    && (fabsf(chassis_motion.chassis_target_omega) < CHASSIS_ZERO_EPS_W)
#endif
                )
			{
				chassis_motion.chassis_target_vx = 0.0f;
				chassis_motion.chassis_target_vy = 0.0f;
				chassis_motion.chassis_target_omega = 0.0f;
#if BOARD_ROLE == BOARD_LOWER
                {
                    uint8_t i;
                    for (i = 0; i < 4; ++i) PID_Clear_Error(&chassis[i]);
                    gimbal_vx_applied = gimbal_vy_applied = chassis_omega_applied = 0.0f;
                    for (i = 0; i < 4; ++i)
                    {
                        chassis_motion.wheel_target_omega[i] = 0;
                        chassis_motion.motor_output_value[i] = 0;
                        chassis_last_command[i] = 0;
                    }
                }
#endif
				CAN_cmd_chassis(0, 0, 0, 0);
			}

			else
			{
#if BOARD_ROLE == BOARD_LOWER
                lower_diag.braking = stop_requested;
#endif
				PID_Set_Values(&chassis[0], chassis_motion.wheel_target_omega[0], chassis_motion.wheel_now_omega[0]);
				PID_Set_Values(&chassis[1], chassis_motion.wheel_target_omega[1], chassis_motion.wheel_now_omega[1]);
				PID_Set_Values(&chassis[2], chassis_motion.wheel_target_omega[2], chassis_motion.wheel_now_omega[2]);
				PID_Set_Values(&chassis[3], chassis_motion.wheel_target_omega[3], chassis_motion.wheel_now_omega[3]);

#if BOARD_ROLE == BOARD_LOWER
                {
                    uint8_t i;
                    for (i = 0; i < 4; ++i)
                    {
                        if (lower_diag.stale_wheel_mask & (1u << i))
                        {
                            PID_Clear_Error(&chassis[i]);
                            chassis_motion.motor_output_value[i] = 0;
                            chassis_last_command[i] = 0;
                            continue;
                        }
                        chassis_speed_pid(&chassis[i], &chassis_motion.motor_output_value[i]);
                        if (fabsf(chassis_motion.motor_output_value[i]) >= chassis[i].Out_Max)
                            ++lower_diag.wheel_saturation_count[i];
                    }
                }
#else
				PID_Calculate(&chassis[0], &chassis_motion.motor_output_value[0]);
				PID_Calculate(&chassis[1], &chassis_motion.motor_output_value[1]);
				PID_Calculate(&chassis[2], &chassis_motion.motor_output_value[2]);
				PID_Calculate(&chassis[3], &chassis_motion.motor_output_value[3]);
#endif

				output_map(&chassis_motion.motor_output_value[0]);
				output_map(&chassis_motion.motor_output_value[1]);
				output_map(&chassis_motion.motor_output_value[2]);
				output_map(&chassis_motion.motor_output_value[3]);
#if BOARD_ROLE == BOARD_LOWER
                {
                    uint8_t i;
                    for (i = 0; i < 4; ++i)
                    {
                        int16_t command = (int16_t)chassis_motion.motor_output_value[i];
                        if ((int32_t)command * chassis_last_command[i] < 0)
                            ++lower_diag.wheel_output_sign_flip_count[i];
                        chassis_last_command[i] = command;
                    }
                }
#endif
					CAN_cmd_chassis((int16_t)chassis_motion.motor_output_value[0], (int16_t)chassis_motion.motor_output_value[1], 
								(int16_t)chassis_motion.motor_output_value[2], (int16_t)chassis_motion.motor_output_value[3]);
			}
				osDelayUntil(&xLastWakeTime,xFrequency);
		}
			    
}

static void gimbal_coordinate_resolution(float theta, chassis_motion_value_t *chassis_motion)
{   
    float32_t sin_theta, cos_theta;
    float vx = chassis_motion->gimbal_target_vx;
    float vy = chassis_motion->gimbal_target_vy;
    arm_sin_cos_f32(theta, &sin_theta, &cos_theta); 
#if BOARD_ROLE == BOARD_LOWER
    /* Limit operator translation in the gun frame, BEFORE rotating the basis.
     * Body-frame components must follow theta during spin; their changing
     * coordinates are not a fresh acceleration request from the operator. */
    control_slew_vector(vx, vy,
                        &gimbal_vx_applied, &gimbal_vy_applied, CHASSIS_LINEAR_ACCEL_MPS2 * 0.001f);
    vx = gimbal_vx_applied;
    vy = gimbal_vy_applied;
    chassis_omega_applied = control_slew(chassis_motion->chassis_target_omega, chassis_omega_applied,
                                        CHASSIS_ANGULAR_ACCEL_RPS2 * 0.001f);
    chassis_motion->chassis_target_omega = chassis_omega_applied;
#endif
    chassis_motion->chassis_target_vx = cos_theta * vx - sin_theta * vy;
    chassis_motion->chassis_target_vy = sin_theta * vx + cos_theta * vy;
    chassis_wheel_inverse_resolution(chassis_motion);
}

static void chassis_wheel_inverse_resolution(chassis_motion_value_t *chassis_motion)
{
    chassis_motion->wheel_target_omega[0] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //1
    chassis_motion->wheel_target_omega[1] = GEAR_RATIO * (0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //2
    chassis_motion->wheel_target_omega[2] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx + 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //3
    chassis_motion->wheel_target_omega[3] = GEAR_RATIO * (-0.707f * chassis_motion->chassis_target_vx - 0.707f * chassis_motion->chassis_target_vy - chassis_motion->chassis_target_omega * WHEEL_TO_CORE_DISTANCE) / WHEEL_RADIUS; //4
}

static void theta_calculate(chassis_motion_value_t *chassis_motion)
{
#if BOARD_ROLE == BOARD_LOWER
    chassis_motion->theta = control_encoder_yaw((uint16_t)chassis_motion->now_ecd,
                                               CHASSIS_YAW_ZERO_ECD, CHASSIS_YAW_ECD_SIGN);
#else
    chassis_motion->theta = (chassis_motion->now_ecd - 6128.0f) * 360.0f / 8192.0f;
#endif
}

static void output_map(float *value)
{
    *value *= 819.2f; 
}

static void km_control(void)
{
	chassis_motion.gimbal_target_vx = (key.key_w - key.key_s) * 1.5f;
	chassis_motion.gimbal_target_vy = (key.key_a - key.key_d) * 1.5f;

	/* F 键按下瞬间切换底盘跟随开关 */
	if (key.key_f && !last_key_f)
	{
		chassis_follow_enabled ^= 1u;
		PID_Clear_Error(&follow_pid);
	}
	last_key_f = key.key_f;

	if (key.key_shift)
	{
#if BOARD_ROLE == BOARD_LOWER
        chassis_motion.chassis_target_omega = CHASSIS_SPIN_OMEGA_RPS;
#else
		chassis_motion.chassis_target_omega = 2 * PI; /* 小陀螺优先 */
#endif
	}
	else if (!chassis_follow_enabled)
	{
		chassis_motion.chassis_target_omega = 0.0f;
	}
	/* 跟随模式下 omega 由主循环中的 follow_pid 写入，此处不赋值 */
}

static void rc_control(void)
{
#if BOARD_ROLE == BOARD_LOWER
    uint16_t channels[2] = {key.ch0, key.ch1};
    chassis_motion.gimbal_target_vx = CHASSIS_RC_FORWARD_SIGN * chassis_rc_axis(channels[CHASSIS_RC_FORWARD_CHANNEL]);
    chassis_motion.gimbal_target_vy = CHASSIS_RC_LEFT_SIGN * chassis_rc_axis(channels[CHASSIS_RC_LEFT_CHANNEL]);
#else
	int16_t ch0_delta = rc_delta_with_deadband(key.ch0);
	int16_t ch1_delta = rc_delta_with_deadband(key.ch1);

	chassis_motion.gimbal_target_vx = ch0_delta / 660.0f;
	chassis_motion.gimbal_target_vy = -ch1_delta / 660.0f;
#endif

	/* ch9（四拨杆）电平直读：下拨 btn_l=1 开跟随，上拨 btn_l=0 关跟随 */
	if (chassis_follow_enabled != key.btn_l)
	{
		chassis_follow_enabled = key.btn_l;
		if (!chassis_follow_enabled)
		{
			PID_Clear_Error(&follow_pid);
		}
	}

	if (key.pause != 0u)
	{
#if BOARD_ROLE == BOARD_LOWER
        /* The original ch7 switch adds spin; ch0/ch1 translation stays active. */
        chassis_motion.chassis_target_omega = CHASSIS_SPIN_OMEGA_RPS;
#else
		chassis_motion.chassis_target_omega = 2.0f * PI; /* 小陀螺优先 */
#endif
	}
	else if (!chassis_follow_enabled)
	{
		chassis_motion.chassis_target_omega = 0.0f;
	}
	/* 跟随模式下 omega 由主循环中的 follow_pid 写入，此处不赋值 */
}
