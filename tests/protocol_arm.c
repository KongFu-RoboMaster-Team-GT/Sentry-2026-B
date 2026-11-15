/* Actual receiver, CAN routing, transport, PID and local chassis task under ARM. */
#include "../User/board_link.c"
#include "../User/Middleware/drv_CAN_receive.c"
#include "../User/remote_control.c"
#define CONTROL_TASK_H
#define CALIBRATE_TASK_H
#include "../User/Algorithm/alg_pid.c"
#include "../Task/chassis_task.c"
#include "../User/Algorithm/SMC.c"
#include "../User/Algorithm/ZeroCheck.c"
/* Keep hardware/USB/INS outside this unit test, run the actual gimbal task. */
#define GIMBAL_TASK_H
#define MOUSE_X_MAP (-1.0f)
#define MOUSE_Y_MAP (-1.0f)
typedef struct { float YAW, PITCH, g_smc[3]; } test_mahony_t;
#include "../User/PC/motion.h"
typedef struct {
    float target_yaw, target_pitch, yaw_output, pitch_output, measured_yaw, yaw_error;
} gimbal_t;
test_mahony_t mahony;
usb_t usb;
#define rc_control test_gimbal_rc_control
#define km_control test_gimbal_km_control
#define rc_delta_with_deadband test_gimbal_rc_delta_with_deadband
#include "../Task/gimbal_task.c"
#undef rc_control
#undef km_control
#undef rc_delta_with_deadband
#if BOARD_ROLE == BOARD_UPPER
#include "../User/Heat/heat_control.c"
#define output_map test_shooter_output_map
#define rc_control test_shooter_rc_control
#define km_control test_shooter_km_control
#include "../Task/shooter_task.c"
#undef output_map
#undef rc_control
#undef km_control
motion_t motion;
Referee_System_t referee;
#endif

CAN_TypeDef test_can1 = {0, 0, 0, CAN_TSR_TME}, test_can2 = {0, 0, 0, CAN_TSR_TME};
CAN_HandleTypeDef hcan1 = {CAN1, 0, HAL_CAN_STATE_LISTENING}, hcan2 = {CAN2, 0, HAL_CAN_STATE_LISTENING};
VTM_Data_t key;
test_calibrate_t calibrate;
static USART_TypeDef test_uart;
UART_HandleTypeDef huart3 = {&test_uart};
DMA_HandleTypeDef hdma_usart3_rx;
uint32_t test_uart_idle, test_dma_remaining;
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { return HAL_OK; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *u) {}

volatile uint32_t test_failed_line, test_assertions;
static uint32_t tick, tx_count, last_id, mailbox_free = 3;
static CAN_HandleTypeDef *last_bus;
static uint8_t last_data[8], sent_input[8], sent_yaw[8], sent_chassis[8];
static uint32_t input_tx_count, yaw_tx_count, chassis_tx_count;
static uint8_t test_mailbox_consumes;
static uint32_t last_abort_mask;
static uint8_t rx_pending, rx_data[8], test_local_loop, test_gimbal_loop;
static CAN_RxHeaderTypeDef rx_header;
static int rx_status = HAL_OK;
static uint32_t abort_count[2], notify_count[2];
static int abort_status = HAL_OK, notify_status = HAL_OK;
static uint32_t local_steps, local_tx_start;
#if BOARD_ROLE == BOARD_UPPER
static uint32_t gimbal_steps, gimbal_previous_tick;
static uint32_t shooter_steps;
static uint8_t test_shooter_loop;
#endif
#if BOARD_ROLE == BOARD_LOWER
static uint32_t local_input_loss_start;
static uint8_t test_chassis_stop_loop;
static uint32_t chassis_stop_steps;
static uint8_t test_chassis_spin_loop, spin_sequence;
static uint32_t spin_steps;
static uint16_t spin_flags, spin_ch1, spin_encoder;
static float previous_spin_omega;
static void input_frame(uint8_t seq, uint16_t flags, uint8_t corrupt);
static void input_frame_xy(uint8_t seq, uint16_t flags, uint8_t corrupt, uint16_t ch0, uint16_t ch1);
#endif

void test_done(void) { for (;;) {} }
#define CHECK(expr) do { ++test_assertions; if (!(expr)) { test_failed_line = __LINE__; test_done(); } } while (0)
void *memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = dst; const uint8_t *s = src;
    while (n--) *d++ = *s++;
    return dst;
}
void *memset(void *dst, int value, size_t n)
{
    uint8_t *d = dst; while (n--) *d++ = (uint8_t)value; return dst;
}
void __aeabi_memclr(void *p, size_t n)
{
    volatile uint8_t *d = p; while (n--) *d++ = 0;
}
void __aeabi_memclr4(void *p, size_t n) { __aeabi_memclr(p, n); }
void __aeabi_memcpy(void *p, const void *q, size_t n)
{
    volatile uint8_t *d = p; const volatile uint8_t *s = q;
    while (n--) *d++ = *s++;
}
void __aeabi_memcpy4(void *d, const void *s, size_t n) { __aeabi_memcpy(d, s, n); }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t osKernelSysTick(void) { return tick; }
void osDelay(uint32_t n)
{
    tick += n;
#if BOARD_ROLE == BOARD_UPPER
    if (test_gimbal_loop) { board_note_input(); gimbal_previous_tick = tick; }
#endif
}
void arm_sin_cos_f32(float theta, float *s, float *c)
{
    float angle = control_wrap_deg(theta);
    if (control_abs(angle) < 0.0001f) { *s = 0; *c = 1; }
    else if (control_abs(angle - 90.0f) < 0.0001f) { *s = 1; *c = 0; }
    else if (control_abs(angle + 90.0f) < 0.0001f) { *s = -1; *c = 0; }
    else if (control_abs(angle + 180.0f) < 0.0001f) { *s = 0; *c = -1; }
    else CHECK(0); /* These tests deliberately use cardinal angles only. */
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can)
{
    uint32_t i, n = 0;
    if (!test_mailbox_consumes || can != &hcan1) return mailbox_free;
    for (i = 0; i < 3; ++i) if (can->Instance->TSR & (CAN_TSR_TME0 << i)) ++n;
    return n;
}
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can, uint32_t mailboxes)
{
    ++abort_count[can->Instance == CAN1 ? 0 : 1];
    CHECK(mailboxes != 0 && (mailboxes & ~7u) == 0);
    last_abort_mask = mailboxes;
    return abort_status;
}
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t notifications)
{
    ++notify_count[can->Instance == CAN1 ? 0 : 1];
    CHECK(notifications == CAN_IT_RX_FIFO0_MSG_PENDING);
    return notify_status;
}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can, CAN_TxHeaderTypeDef *h, uint8_t *data, uint32_t *box)
{
    uint32_t slot = 0;
    if (test_mailbox_consumes && can == &hcan1)
    {
        while (slot < 3 && !(can->Instance->TSR & (CAN_TSR_TME0 << slot))) ++slot;
        if (slot == 3) return 1;
        can->Instance->TSR &= ~(CAN_TSR_TME0 << slot);
    }
    ++tx_count; last_id = h->StdId; last_bus = can;
    memcpy(last_data, data, 8); *box = 1u << slot;
    if (can == &hcan1 && h->StdId == BOARD_CHASSIS_INPUT_ID)
    { memcpy(sent_input, data, 8); ++input_tx_count; }
    if (can == &hcan1 && h->StdId == BOARD_YAW_COMMAND_ID)
    { memcpy(sent_yaw, data, 8); ++yaw_tx_count; }
    if (can == &hcan2 && h->StdId == 0x200)
    { memcpy(sent_chassis, data, 8); ++chassis_tx_count; }
    return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *can, uint32_t fifo) { return rx_pending; }
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *can, uint32_t fifo, CAN_RxHeaderTypeDef *h, uint8_t *data)
{
    if (rx_status != HAL_OK) { rx_pending = 0; return rx_status; }
    *h = rx_header; memcpy(data, rx_data, 8); rx_pending = 0; return HAL_OK;
}
static void feedback(CAN_HandleTypeDef *can, uint32_t id, int16_t speed)
{
    rx_header.StdId = id; rx_header.IDE = CAN_ID_STD;
    rx_header.RTR = CAN_RTR_DATA; rx_header.DLC = 8;
    memset(rx_data, 0, 8);
#if BOARD_ROLE == BOARD_LOWER
    put16(rx_data, CHASSIS_YAW_ZERO_ECD);
#else
    put16(rx_data, 6128);
#endif
    put16(rx_data + 2, speed);
    rx_pending = 1; HAL_CAN_RxFifo0MsgPendingCallback(can);
}
#if BOARD_ROLE == BOARD_LOWER
static void feedback_yaw_ecd(uint16_t encoder)
{
    rx_header.StdId = 0x205; rx_header.IDE = CAN_ID_STD;
    rx_header.RTR = CAN_RTR_DATA; rx_header.DLC = 8;
    memset(rx_data, 0, 8); put16(rx_data, encoder);
    rx_pending = 1; HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
}
#endif
void osDelayUntil(uint32_t *wake, uint32_t period)
{
    *wake += period; tick = *wake;
    if (test_local_loop)
    {
        ++local_steps;
        CHECK(period == 1);
        CHECK(chassis_tx_count == local_tx_start + local_steps);
#if BOARD_ROLE == BOARD_LOWER
        CHECK(lower_diag.chassis_loop_count == local_steps);
        CHECK(lower_diag.ch1 == 1354 && lower_diag.mode == 0);
        CHECK(lower_diag.chassis_command[0] == get16(sent_chassis));
        CHECK(lower_diag.diagnostic_version == 1 && lower_diag.control_max_gap_ms <= 1);
        if (local_steps >= 102 && local_steps <= 110)
        {
            CHECK(lower_diag.input_reason == 3 && lower_diag.last_input_loss_reason == 3);
            CHECK(lower_diag.input_loss_count == local_input_loss_start + 1);
            CHECK(lower_diag.last_input_loss_age_ms == 101);
        }
        if (local_steps == 111)
        {
            CHECK(lower_diag.input_reason == 0 && lower_diag.last_input_loss_reason == 3);
            CHECK(lower_diag.input_loss_count == local_input_loss_start + 1);
        }
        if (local_steps == 120) CHECK(lower_diag.wheel_output_sign_flip_count[0] > 10);
#endif
        CHECK(chassis[0].K_P == 0.95f && chassis[0].K_I == 0.1f && chassis[0].K_D == 0.0f);
        CHECK(control_abs(chassis_motion.gimbal_target_vx -
                          (310.0f / 763.0f * (783.0f / 660.0f))) < 0.00001f);
        if (local_steps >= 102 && local_steps <= 110)
        {
            CHECK(get16(sent_chassis) == 0);
            CHECK(chassis[0].Integral_Error == 0.0f);
        }
        else if (local_steps & 1u) CHECK(get16(sent_chassis) > 0);
        else CHECK(get16(sent_chassis) < 0);
        /* New local feedback arrives every ms; NO new CAN1 input is injected.
         * This verifies the real PID reacts before any 10/20ms transport cycle. */
        feedback(&hcan2, 0x201, (local_steps & 1u) ? 2000 : 0);
#if BOARD_ROLE == BOARD_LOWER
        if (local_steps == 110) input_frame_xy(41, 0x0800, 0, 1024, 1354);
#endif
        if (local_steps >= 120)
        {
#if BOARD_ROLE == BOARD_LOWER
            uint8_t i;
            test_local_loop = 0; test_chassis_stop_loop = 1;
            input_frame_xy(42, 0x0800, 0, 1024, 1024);
            gimbal_vx_applied = gimbal_vy_applied = chassis_omega_applied = 0;
            for (i = 0; i < 4; ++i) feedback(&hcan2, 0x201 + i, i == 0 ? 200 : 0);
            return;
#else
            test_done();
#endif
        }
    }
#if BOARD_ROLE == BOARD_LOWER
    if (test_chassis_stop_loop)
    {
        ++chassis_stop_steps;
        if (chassis_stop_steps == 1)
        {
            CHECK(get16(sent_chassis) < 0 && lower_diag.braking);
            feedback(&hcan2, 0x201, 0);
        }
        else if (chassis_stop_steps == 2)
        {
            CHECK(get16(sent_chassis) == 0 && !lower_diag.braking);
            CHECK(lower_diag.zero_request_count == 1);
            CHECK(chassis_motion.wheel_target_omega[0] == 0);
            CHECK(chassis_motion.motor_output_value[0] == 0 && chassis[0].Integral_Error == 0);
            feedback(&hcan2, 0x201, 200); /* Neutral feedback disturbance must not re-arm. */
        }
        else if (chassis_stop_steps == 3)
        {
            CHECK(get16(sent_chassis) == 0 && !lower_diag.braking);
            feedback(&hcan2, 0x201, 0);
            input_frame_xy(43, 0x0800, 0, 1024, 1354);
        }
        else if (chassis_stop_steps == 4)
        {
            CHECK(get16(sent_chassis) > 0);
            /* Only motor 1 loses feedback; other wheels remain controllable. */
            feedback_tick[0] = tick - BOARD_MOTOR_TIMEOUT_MS - 1;
        }
        else if (chassis_stop_steps == 5)
        {
            CHECK(lower_diag.stale_wheel_mask == 1 && get16(sent_chassis) == 0);
            CHECK(lower_diag.feedback_stale_count[0] == 1);
            CHECK(get16(sent_chassis + 2) > 0 && chassis[0].Integral_Error == 0);
            feedback(&hcan2, 0x201, 0);
        }
        else if (chassis_stop_steps == 6)
        {
            CHECK(lower_diag.stale_wheel_mask == 0 && get16(sent_chassis) > 0);
            CHECK(lower_diag.feedback_stale_count[0] == 1 && lower_diag.feedback_max_gap_ms[0] > 100);
            input_frame_xy(44, 0, 0, 1024, 1354);
            feedback(&hcan2, 0x201, 200);
        }
        else if (chassis_stop_steps == 7)
        {
            CHECK(get16(sent_chassis) == 0 && get16(sent_chassis + 2) == 0);
            CHECK(chassis[0].Integral_Error == 0); /* Link loss bypasses braking. */
            CHECK(lower_diag.input_reason == 2 && lower_diag.last_input_loss_reason == 2);
            CHECK(lower_diag.input_loss_count == local_input_loss_start + 2);
            input_frame_xy(45, 0x0800, 0, 1024, 1354);
            {
                uint8_t i;
                for (i = 0; i < 4; ++i) feedback(&hcan2, 0x201 + i, 0);
            }
        }
        else
        {
            uint8_t i;
            /* Hold the stick for 1s with fresh 10ms CAN1 input and 1ms CAN2
             * feedback. No autonomous release or intermittent zero is allowed.
             * The motor plant is NOT simulated; this tests gating/timing only. */
            CHECK(lower_diag.input_reason == 0 && lower_diag.motion_enabled);
            CHECK(lower_diag.input_loss_count == local_input_loss_start + 2);
            CHECK(!lower_diag.idle_latched && !lower_diag.braking);
            CHECK(lower_diag.stale_wheel_mask == 0);
            CHECK(get16(sent_chassis) > 0 && get16(sent_chassis + 2) > 0);
            CHECK(get16(sent_chassis + 4) < 0 && get16(sent_chassis + 6) < 0);
            for (i = 0; i < 4; ++i) feedback(&hcan2, 0x201 + i, 0);
            if ((chassis_stop_steps - 7) % 10 == 0)
                input_frame_xy((uint8_t)(45 + (chassis_stop_steps - 7) / 10), 0x0800, 0, 1024, 1354);
            if (chassis_stop_steps >= 1007)
            {
                test_chassis_stop_loop = 0; test_chassis_spin_loop = 1;
                spin_sequence = 146; spin_flags = 0x080Cu; spin_ch1 = 1354;
                spin_encoder = CHASSIS_YAW_ZERO_ECD;
                previous_spin_omega = 0;
                follow_pid.Integral_Error = 5;
                input_frame_xy(spin_sequence, spin_flags, 0, 1024, spin_ch1);
                feedback_yaw_ecd(spin_encoder);
                return;
            }
        }
    }
    if (test_chassis_spin_loop)
    {
        uint8_t i;
        float forward = chassis_rc_axis(1354);
        ++spin_steps;
        CHECK(period == 1);
        if (spin_steps <= 2101)
        {
            CHECK(lower_diag.input_reason == 0 && lower_diag.stale_wheel_mask == 0);
            CHECK(lower_diag.input_loss_count == local_input_loss_start + 2);
            CHECK(control_abs(chassis_motion.chassis_target_omega - previous_spin_omega) <=
                  CHASSIS_ANGULAR_ACCEL_RPS2 * 0.001f + 0.00001f);
            previous_spin_omega = chassis_motion.chassis_target_omega;
        }
        if (spin_steps <= 900)
        {
            CHECK(!lower_diag.idle_latched && !lower_diag.braking);
            CHECK(chassis_follow_enabled && follow_pid.Integral_Error == 0); /* Spin beats follow. */
            CHECK(control_abs(chassis_motion.gimbal_target_vx - forward) < 0.00001f);
            if (spin_steps == 900)
            {
                CHECK(chassis_motion.chassis_target_omega == CHASSIS_SPIN_OMEGA_RPS);
                spin_encoder = CHASSIS_YAW_ZERO_ECD + 2048;
            }
        }
        if (spin_steps >= 901 && spin_steps <= 904)
        {
            float expected_x[4] = {0, -1, 0, 1};
            float expected_y[4] = {1, 0, -1, 0};
            uint16_t next_encoder[4] = {CHASSIS_YAW_ZERO_ECD + 4096,
                CHASSIS_YAW_ZERO_ECD - 2048, CHASSIS_YAW_ZERO_ECD, CHASSIS_YAW_ZERO_ECD};
            uint8_t direction = (uint8_t)(spin_steps - 901);
            CHECK(control_abs(chassis_motion.chassis_target_vx - expected_x[direction] * forward) < 0.0001f);
            CHECK(control_abs(chassis_motion.chassis_target_vy - expected_y[direction] * forward) < 0.0001f);
            CHECK(chassis_motion.chassis_target_omega == CHASSIS_SPIN_OMEGA_RPS);
            CHECK(!lower_diag.idle_latched);
            spin_encoder = next_encoder[direction];
        }
        if (spin_steps == 905)
        {
            spin_ch1 = 1024; spin_flags = 0x0804u;
            input_frame_xy(++spin_sequence, spin_flags, 0, 1024, spin_ch1);
        }
        if (spin_steps == 1300)
        {
            CHECK(chassis_motion.chassis_target_vx == 0 && chassis_motion.chassis_target_vy == 0);
            CHECK(chassis_motion.chassis_target_omega == CHASSIS_SPIN_OMEGA_RPS);
            CHECK(!lower_diag.idle_latched); /* Centered sticks still spin. */
            for (i = 1; i < 4; ++i)
                CHECK(chassis_motion.wheel_target_omega[i] == chassis_motion.wheel_target_omega[0]);
            spin_flags = 0x0800u;
        }
        if (spin_steps == 2100)
        {
            CHECK(lower_diag.idle_latched && chassis_motion.chassis_target_omega == 0);
            for (i = 0; i < 4; ++i) CHECK(get16(sent_chassis + 2 * i) == 0);
            spin_flags = 0x0804u; spin_ch1 = 1354;
        }
        if (spin_steps == 2101)
        {
            CHECK(!lower_diag.idle_latched && chassis_motion.chassis_target_omega > 0);
            input_frame_xy(++spin_sequence, 0x0004u, 0, 1024, spin_ch1);
        }
        if (spin_steps == 2102)
        {
            CHECK(lower_diag.input_reason == 2 && lower_diag.idle_latched);
            CHECK(lower_diag.input_loss_count == local_input_loss_start + 3);
            CHECK(gimbal_vx_applied == 0 && gimbal_vy_applied == 0 && chassis_omega_applied == 0);
            for (i = 0; i < 4; ++i)
                CHECK(get16(sent_chassis + 2 * i) == 0 && chassis[i].Integral_Error == 0);
            test_done();
        }
        for (i = 0; i < 4; ++i) feedback(&hcan2, 0x201 + i, 0);
        feedback_yaw_ecd(spin_encoder);
        if (spin_steps % 10 == 0)
            input_frame_xy(++spin_sequence, spin_flags, 0, 1024, spin_ch1);
    }
#endif
#if BOARD_ROLE == BOARD_UPPER
    if (test_shooter_loop)
    {
        ++shooter_steps;
        CHECK(last_bus == &hcan1 && last_id == 0x200);
        CHECK(get16(last_data + 6) == 0);
        CHECK(shooter[0].K_P == 0.15f && shooter[0].K_I == 0.3f && shooter[0].K_D == 0.002f);
        CHECK(shooter[1].Out_Max == 10 && shooter[2].K_P == 0.1f && shooter[2].K_I == 0.02f);
        if (shooter_steps == 1 || shooter_steps == 4 || shooter_steps == 6 || shooter_steps == 207)
        {
            CHECK(get16(last_data) == 0 && get16(last_data + 2) == 0 && get16(last_data + 4) == 0);
        }
        else
        {
            CHECK(get16(last_data) < 0 && get16(last_data + 2) > 0 && get16(last_data + 4) > 0);
        }
        if (shooter_steps == 1) rc_ctrl.rc.ch[8] = -1;
        if (shooter_steps == 2)
        {
            CHECK(get16(last_data) == -8192 && get16(last_data + 2) == 8192);
            CAN_cmd_pitch(500);
            CHECK(last_id == 0x1FE && get16(last_data + 2) == 500);
        }
        if (shooter_steps == 3) calibrate.flag = 1;
        if (shooter_steps == 4) { CHECK(shooter_control.shoot_flag == 1); calibrate.flag = 0; }
        if (shooter_steps == 5 || shooter_steps == 7) rc_ctrl.rc.ch[8] = 0;
        if (shooter_steps == 6) rc_ctrl.rc.ch[8] = 1;
        board_note_input();
        if (shooter_steps == 207)
        {
            test_shooter_loop = 0;
            tick = 3000; key.sw = 1; key.ch2 = key.ch3 = 1024;
            key.mouse_x = key.mouse_y = 0; key.mouse_btn_r = 0;
            calibrate.flag = 0; mahony.YAW = 179; mahony.PITCH = 0;
            mahony.g_smc[0] = mahony.g_smc[1] = mahony.g_smc[2] = 0;
            board_note_input(); test_gimbal_loop = 1;
            GIMBAL_TASK(0);
        }
        return;
    }
    if (test_gimbal_loop)
    {
        ++gimbal_steps;
        CHECK(tick == gimbal_previous_tick + 1u); /* No post-delay catch-up burst. */
        gimbal_previous_tick = tick;
        if (gimbal_steps != 64) CHECK(control_abs(yaw.error.tar_differential) < 0.001f);
        if (gimbal_steps == 1)
        {
            CHECK(yaw.param.J == 0.8f && yaw.param.K == 120 && yaw.param.c == 8);
            CHECK(control_abs(gimbal.yaw_output) < 0.001f);
            mahony.YAW = -179.0f;
        }
        else if (gimbal_steps == 2)
        {
            CHECK(control_abs(gimbal.measured_yaw - 181.0f) < 0.001f);
            CHECK(control_abs(gimbal.yaw_error + 2.0f) < 0.001f);
            CHECK(gimbal.yaw_output < -120.0f); /* No old positive command during braking. */
            CHECK(control_abs(gimbal.yaw_output) <= YAW_OUTPUT_LIMIT);
            CHECK(yaw_diag.raw_output == yaw.u && yaw_diag.fault == YAW_FAULT_NONE);
            mahony.YAW = 179.0f;
        }
        else if (gimbal_steps == 3)
        {
            CHECK(control_abs(gimbal.yaw_output) < 0.001f);
            key.sw = 0; mahony.YAW = -179.0f;
        }
        else if (gimbal_steps == 4)
        {
            CHECK(control_abs(gimbal.yaw_output) < 0.001f); /* Bumpless mode change. */
            calibrate.flag = 1; mahony.YAW = 90.0f;
        }
        else if (gimbal_steps == 5)
        {
            CHECK(gimbal.yaw_output == 0 && yaw_command == 0);
            CHECK(last_id == 0x200 && get16(last_data) == 0 &&
                  get16(last_data + 2) == 0 && get16(last_data + 4) == 0);
            calibrate.flag = 0;
        }
        else if (gimbal_steps == 6)
        {
            CHECK(control_abs(gimbal.yaw_output) < 0.001f);
            mahony.YAW = 150.0f; key.ch2 = 1354;
        }
        else if (gimbal_steps >= 7 && gimbal_steps <= 56)
        {
            CHECK(yaw_diag.fault == YAW_FAULT_NONE);
            CHECK(gimbal.yaw_output == -YAW_OUTPUT_LIMIT);
        }
        else if (gimbal_steps == 57)
        {
            uint32_t input_count_before = input_tx_count;
#if YAW_MOTION_GUARD_ENABLE
            CHECK(yaw_diag.fault == YAW_FAULT_TRACKING);
            CHECK(gimbal.yaw_output == 0 && yaw_command == 0);
            CHECK(yaw_diag.fault_error_deg == -60 && yaw_diag.fault_rate_dps == 0);
#else
            CHECK(yaw_diag.fault == YAW_FAULT_NONE);
            CHECK(gimbal.yaw_output == -YAW_OUTPUT_LIMIT && yaw_command == -16384);
#endif
            CHECK(gimbal.pitch_output != 0 && get16(last_data + 2) != 0); /* Pitch remains independent. */
            board_link_poll();
            CHECK(board_motion_enabled() && board_armed == 1);
            CHECK(input_tx_count == input_count_before + 1);
            CHECK(get16(sent_input + 2) == (int16_t)key.ch1 && key.ch1 != 1024);
            CHECK(((uint16_t)get16(sent_input + 4) & 0x0800u) != 0);
            CHECK(get16(sent_yaw) == yaw_command && sent_yaw[2] == 1);
            mahony.YAW = 90;
        }
        else if (gimbal_steps == 58)
        {
#if YAW_MOTION_GUARD_ENABLE
            CHECK(yaw_diag.fault == YAW_FAULT_TRACKING && gimbal.yaw_output == 0);
            CHECK(yaw_diag.fault_error_deg == -60); /* Snapshot survives later normal data. */
#else
            CHECK(yaw_diag.fault == YAW_FAULT_NONE && gimbal.yaw_output == 0);
#endif
            key.sw = 1;
        }
        else if (gimbal_steps == 59)
        {
            CHECK(yaw_diag.fault == YAW_FAULT_NONE && gimbal.yaw_output == 0);
            mahony.YAW = 95;
        }
        else if (gimbal_steps == 60)
        {
            CHECK(yaw_diag.fault == YAW_FAULT_NONE);
            CHECK(gimbal.yaw_output < -3800 && gimbal.yaw_output > -3900);
            mahony.YAW = 85;
        }
        else if (gimbal_steps == 61)
        {
            CHECK(yaw_diag.fault == YAW_FAULT_NONE);
            CHECK(gimbal.yaw_output > 3800 && gimbal.yaw_output < 3900);
            CHECK(yaw_command == (int16_t)gimbal.yaw_output); /* Immediate reversal in the next iteration. */
            mahony.YAW = 90;
        }
        else if (gimbal_steps == 62)
        {
            CHECK(gimbal.yaw_output == 0 && yaw_command == 0);
            /* Synchronous observation corrected the old -1 gyro sign. */
            mahony.YAW = 90.1f; mahony.g_smc[2] = 1.0f;
        }
        else if (gimbal_steps == 63)
        {
            CHECK(yaw_diag.gyro_z_dps > 0 && yaw_diag.rate_dps > 0);
            CHECK(yaw.error.v_error > 0 && gimbal.yaw_output < 0);
            CHECK(control_abs(gimbal.yaw_output) <= YAW_OUTPUT_LIMIT);
            key.sw = 0; key.ch3 = 1807; mahony.YAW = 90; mahony.g_smc[2] = 0;
        }
        else if (gimbal_steps == 64)
        {
            CHECK(yaw_diag.fault == 0 && yaw.error.tar_differential < -119.9f && yaw.error.tar_differential > -120.1f);
            CHECK(gimbal.target_yaw < 89.881f && gimbal.target_yaw > 89.879f);
            mahony.YAW = 89.99f; key.ch3 = 1022;
        }
        else if (gimbal_steps == 65)
        {
            CHECK(gimbal.target_yaw == mahony.YAW); /* Release latches the current heading. */
            mahony.YAW = 90.99f;
        }
        else if (gimbal_steps == 66)
        {
            union { uint32_t bits; float value; } invalid;
            CHECK(control_abs(gimbal.target_yaw - 89.99f) < 0.001f);
            CHECK(gimbal.yaw_error < -0.99f && gimbal.yaw_output < 0 && yaw_diag.fault == 0);
            invalid.bits = 0x7FC00000u;
            mahony.YAW = invalid.value;
        }
        else if (gimbal_steps == 67)
        {
            CHECK(gimbal.yaw_output == 0 && yaw_command == 0);
            CHECK(control_finite(gimbal.pitch_output) && gimbal.pitch_output != 0);
            CHECK(last_id == 0x1FE && get16(last_data + 2) != 0);
            mahony.YAW = 90.99f;
        }
        else if (gimbal_steps == 68)
        {
            union { uint32_t bits; float value; } invalid;
            CHECK(control_finite(gimbal.yaw_output) && control_abs(gimbal.yaw_output) < 0.001f);
            CHECK(gimbal.pitch_output != 0);
            invalid.bits = 0x7FC00000u;
            mahony.g_smc[2] = invalid.value;
        }
        else if (gimbal_steps == 69)
        {
            CHECK(gimbal.yaw_output == 0 && yaw_command == 0);
            CHECK(control_finite(gimbal.pitch_output) && gimbal.pitch_output != 0);
            mahony.g_smc[2] = 0;
            mahony.YAW = 91.99f;
        }
        else if (gimbal_steps == 70)
        {
            union { uint32_t bits; float value; } invalid;
            CHECK(control_finite(gimbal.yaw_output));
            CHECK(control_finite(gimbal.pitch_output) && gimbal.pitch_output != 0);
            invalid.bits = 0x7FC00000u;
            mahony.PITCH = invalid.value;
            mahony.YAW += 1;
        }
        else if (gimbal_steps == 71)
        {
            union { uint32_t bits; float value; } invalid;
            CHECK(gimbal.pitch_output == 0 && get16(last_data + 2) == 0);
            CHECK(control_finite(gimbal.yaw_output) && gimbal.yaw_output < 0);
            invalid.bits = 0x7FC00000u;
            mahony.PITCH = 0; mahony.g_smc[1] = invalid.value;
        }
        else if (gimbal_steps == 72)
        {
            CHECK(gimbal.pitch_output == 0 && get16(last_data + 2) == 0);
            CHECK(control_finite(gimbal.yaw_output) && gimbal.yaw_output < 0);
            mahony.g_smc[1] = 0;
        }
        else if (gimbal_steps >= 73)
        {
            CHECK(control_finite(gimbal.pitch_output) && gimbal.pitch_output != 0);
            CHECK(control_finite(gimbal.yaw_output) && gimbal.yaw_output < 0);
            test_done();
        }
        board_note_input();
    }
#endif
}
static void receiver_frame(uint16_t mode_raw, uint16_t fire_raw)
{
    uint8_t *frame = sbus_rx_buf[rc_rx_buf_index];
    uint16_t channels[10], channel, bit;
    for (channel = 0; channel < 10; ++channel) channels[channel] = 1024;
    channels[6] = mode_raw; channels[8] = fire_raw;
    memset(frame, 0, SBUS_RX_BUF_NUM); frame[0] = 0x0F;
    for (channel = 0; channel < 10; ++channel)
        for (bit = 0; bit < 11; ++bit)
            if (channels[channel] & (1u << bit))
            {
                uint16_t offset = channel * 11 + bit;
                frame[1 + offset / 8] |= (1u << (offset % 8));
            }
    test_uart_idle = 1; test_dma_remaining = SBUS_RX_BUF_NUM - 25;
    RC_USART3_IRQHandler();
}
#if BOARD_ROLE == BOARD_LOWER
static void input_frame(uint8_t seq, uint16_t flags, uint8_t corrupt)
{
    input_frame_xy(seq, flags, corrupt, 1354, 1024);
}
static void input_frame_xy(uint8_t seq, uint16_t flags, uint8_t corrupt, uint16_t ch0, uint16_t ch1)
{
    uint8_t d[8] = {0};
    put16(d, (int16_t)ch0); put16(d + 2, (int16_t)ch1);
    put16(d + 4, (BOARD_PROTOCOL_VERSION << 12) | flags);
    d[6] = seq; d[7] = checksum(d) ^ corrupt;
    rx_header.StdId = BOARD_CHASSIS_INPUT_ID; rx_header.IDE = CAN_ID_STD;
    rx_header.RTR = CAN_RTR_DATA; rx_header.DLC = 8;
    memcpy(rx_data, d, 8); rx_pending = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}
static void yaw_frame(uint8_t seq, uint8_t enable, uint8_t corrupt)
{
    uint8_t d[8] = {0};
    put16(d, -500); d[2] = enable; d[3] = seq;
    d[4] = BOARD_PROTOCOL_VERSION; d[5] = 0x55; d[6] = 0xAA;
    d[7] = checksum(d) ^ corrupt;
    board_link_receive(BOARD_YAW_COMMAND_ID, d);
}
#endif
void test_main(void)
{
    uint32_t count;
    tick = 1000;
    CHECK(board_diag.role == BOARD_ROLE && board_diag.protocol_version == 3);
#if BOARD_ROLE != BOARD_SINGLE
    {
        uint32_t i, queued;
        board_send_motor(&hcan1, 0x555, 111, 0, 0, 0);
        queued = tx_count;
        test_can1.ESR = 0x00F80007u; test_can1.TSR = 0;
        board_send_motor(&hcan1, 0x555, 222, 0, 0, 0);
        CHECK(tx_count == queued); /* Reject a new request as soon as hardware reports Bus-Off. */
        service_can(&hcan1, &can_recovery[0], &board_diag.can1);
        CHECK(board_diag.can1.busoff_count == 1 && board_diag.can1.stage == 1);
        CHECK(board_diag.can1.last_error_esr == 0x00F80007u && abort_count[0] == 1);
        CHECK(test_can1.MCR & CAN_MCR_INRQ);
        for (i = 0; i < 10; ++i) service_can(&hcan1, &can_recovery[0], &board_diag.can1);
        CHECK(board_diag.can1.busoff_count == 1 && abort_count[0] == 1);
        board_send_motor(&hcan2, 0x555, 333, 0, 0, 0);
        CHECK(tx_count == queued + 1); /* The other CAN controller remains usable. */
        test_can1.MSR = CAN_MSR_INAK;
        service_can(&hcan1, &can_recovery[0], &board_diag.can1);
        CHECK(board_diag.can1.stage == 2 && !(test_can1.MCR & CAN_MCR_INRQ));
        service_can(&hcan1, &can_recovery[0], &board_diag.can1);
        CHECK(board_diag.can1.recovered_count == 0); /* Still Bus-Off / init ACK not cleared. */
        test_can1.ESR = 0; test_can1.MSR = 0;
        service_can(&hcan1, &can_recovery[0], &board_diag.can1);
        CHECK(board_diag.can1.stage == 2); /* Abort completion is asynchronous. */
        board_send_motor(&hcan1, 0x555, 444, 0, 0, 0);
        CHECK(tx_count == queued + 1);
        test_can1.TSR = CAN_TSR_TME; notify_status = 1;
        service_can(&hcan1, &can_recovery[0], &board_diag.can1);
        CHECK(board_diag.can1.notify_fail_count == 1 && board_diag.can1.stage == 2);
        notify_status = HAL_OK;
        service_can(&hcan1, &can_recovery[0], &board_diag.can1);
        CHECK(board_diag.can1.recovered_count == 1 && board_diag.can1.stage == 0);
        CHECK(tx_count == queued + 1); /* Recovery did not replay an old payload. */
        board_send_motor(&hcan1, 0x555, 555, 0, 0, 0);
        CHECK(tx_count == queued + 2 && get16(last_data) == 555);
        /* A boot-time HAL_CAN_Start timeout is retried without requiring MCU reset. */
        hcan2.State = HAL_CAN_STATE_ERROR; hcan2.ErrorCode = HAL_CAN_ERROR_TIMEOUT;
        test_can2.MSR = CAN_MSR_INAK; abort_status = 1;
        service_can(&hcan2, &can_recovery[1], &board_diag.can2);
        CHECK(board_diag.can2.start_retry_count == 1 && board_diag.can2.busoff_count == 0);
        CHECK(board_diag.can2.abort_fail_count == 1 && board_diag.can2.stage == 1);
        CHECK(test_can2.MCR & CAN_MCR_INRQ);
        abort_status = HAL_OK;
        service_can(&hcan2, &can_recovery[1], &board_diag.can2);
        CHECK(board_diag.can2.stage == 2 && hcan2.State == HAL_CAN_STATE_LISTENING);
        test_can2.MSR = 0;
        service_can(&hcan2, &can_recovery[1], &board_diag.can2);
        CHECK(board_diag.can2.stage == 0 && board_diag.can2.recovered_count == 1);
        CHECK(hcan2.ErrorCode == 0 && notify_count[1] == 1);
        tx_count = 0; board_tx_fail_count = 0;
#if BOARD_ROLE == BOARD_LOWER
        memset((void *)&lower_diag, 0, sizeof(lower_diag));
#endif
    }
#endif
#if BOARD_ROLE != BOARD_SINGLE
    feedback(&hcan1, 0x201, 0);
    CHECK(board_diag.can1_rx_count == 1 && board_diag.last_can1_id == 0x201);
    rx_header.DLC = 7; rx_pending = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    CHECK(board_diag.can1_rx_count == 2 && board_diag.can1_invalid_frame_count == 1);
    rx_status = 1; rx_pending = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); rx_status = HAL_OK;
    CHECK(board_diag.can1_rx_count == 2 && board_diag.can1_rx_error_count == 1);
#endif
#if BOARD_ROLE == BOARD_UPPER
    {
        uint32_t i;
        float locked, previous;
        SMC_Init(&yaw);
        SMC_SetParam_Exponent(&yaw, YAW_SMC_J, YAW_SMC_K, YAW_SMC_C, YAW_SMC_EPSILON,
                              2.0f, 16384.0f, 0.0f);
        reset_yaw_fault();
        gimbal.measured_yaw = gimbal.target_yaw = 0;
        key.ch2 = 1024; key.ch3 = 1807;
        for (i = 0; i < 1000; ++i)
        {
            ++tick; previous = gimbal.target_yaw;
            test_gimbal_rc_control(&gimbal);
            CHECK(control_abs(gimbal.target_yaw - previous) <= 0.1201f);
            gimbal.measured_yaw = gimbal.target_yaw; /* Ideal tracking, not a plant simulation. */
        }
        CHECK(control_abs(gimbal.target_yaw + 120.0f) < 0.01f); /* Full stick: 120 deg over one second. */
        gimbal.measured_yaw += 0.2f; key.ch3 = 1022; ++tick;
        test_gimbal_rc_control(&gimbal); locked = gimbal.target_yaw;
        CHECK(locked == gimbal.measured_yaw);
        gimbal.measured_yaw += 1.0f;
        for (i = 0; i < 50; ++i)
        {
            ++tick; key.ch3 = (i & 1u) ? 1026 : 1022;
            test_gimbal_rc_control(&gimbal);
            CHECK(gimbal.target_yaw == locked); /* Do not follow a disturbance while centered. */
        }
        SMC_ErrorUpdate(&yaw, locked, gimbal.measured_yaw, 0);
        CHECK(SMC_Calculate(&yaw) < 0); /* Restoring command at the fixed hold reference. */
        reset_yaw_fault();
        gimbal.measured_yaw = gimbal.target_yaw = 0; key.ch3 = 2047;
        ++tick; test_gimbal_rc_control(&gimbal); previous = gimbal.target_yaw;
        SMC_ErrorUpdate(&yaw, gimbal.target_yaw, gimbal.measured_yaw, 0);
        for (i = 0; i < 100; ++i)
        {
            test_gimbal_rc_control(&gimbal);
            SMC_ErrorUpdate(&yaw, gimbal.target_yaw, gimbal.measured_yaw, 0);
        }
        CHECK(gimbal.target_yaw == previous); /* Same tick cannot replay stick displacement. */
        tick += 1000; test_gimbal_rc_control(&gimbal);
        SMC_ErrorUpdate(&yaw, gimbal.target_yaw, gimbal.measured_yaw, 0);
        CHECK(control_abs(gimbal.target_yaw - previous) <= 0.1201f); /* Debug pause is not a one-second turn. */
        for (i = 0; i < 1000; ++i)
        {
            ++tick; test_gimbal_rc_control(&gimbal);
            SMC_ErrorUpdate(&yaw, gimbal.target_yaw, gimbal.measured_yaw, 0);
            CHECK(control_abs(yaw.error.tar_differential) < 120.1f);
        }
        CHECK(control_abs(gimbal.target_yaw) <= 30.001f && control_abs(gimbal.target_yaw) > 29.99f);
        gimbal.measured_yaw = 40; ++tick; test_gimbal_rc_control(&gimbal);
        SMC_ErrorUpdate(&yaw, gimbal.target_yaw, gimbal.measured_yaw, 0);
        CHECK(control_abs(yaw.error.tar_differential) < 120.1f); /* Lead projection has no derivative spike. */
        reset_yaw_fault();
        gimbal.measured_yaw = gimbal.target_yaw = 0; key.ch3 = 0;
        tick = 0xFFFFFFF0u;
        for (i = 0; i < 40; ++i) { ++tick; test_gimbal_rc_control(&gimbal); }
        CHECK(control_abs(gimbal.target_yaw - 4.8f) < 0.001f); /* Reverse stick and clock wrap. */
        reset_yaw_fault(); tick = 1000;
    }
    {
        control_yaw_rate_observer_t observer;
        control_yaw_tracker_t tracker = {0};
        Sliding provisional;
        uint32_t i;
        union { uint32_t bits; float value; } invalid;
        invalid.bits = 0x7FC00000u;
        control_yaw_rate_reset(&observer);
        for (i = 0; i <= 20; ++i)
            control_yaw_rate_observe(&observer,
                control_unwrap_yaw(&tracker, control_wrap_deg(179.0f + (float)i * 0.1f)),
                100.0f, 1000u + i);
        CHECK(observer.direction == YAW_RATE_DIRECTION_SAME);
        CHECK(control_abs(observer.angle_rate_dps - 100.0f) < 0.1f);
        CHECK(control_abs(observer.gyro_mean_dps - 100.0f) < 0.1f);
        for (i = 21; i <= 50; ++i)
            control_yaw_rate_observe(&observer, 181.0f, 0, 1000u + i);
        CHECK(observer.direction == YAW_RATE_DIRECTION_UNKNOWN);
        CHECK(observer.last_direction == YAW_RATE_DIRECTION_SAME);
        CHECK(control_abs(observer.last_angle_rate_dps - 100) < 0.1f);
        CHECK(control_abs(observer.last_gyro_mean_dps - 100) < 0.1f); /* Retained after stopping for Watch. */
        control_yaw_rate_reset(&observer);
        for (i = 0; i <= 20; ++i)
            control_yaw_rate_observe(&observer, (float)i * 0.1f, -100.0f, 1100u + i);
        CHECK(observer.direction == YAW_RATE_DIRECTION_OPPOSITE);
        CHECK(observer.angle_rate_dps > 0 && observer.gyro_mean_dps < 0);
        control_yaw_rate_reset(&observer);
        for (i = 0; i <= 20; ++i)
            control_yaw_rate_observe(&observer, -(float)i * 0.1f, -100.0f, 0xFFFFFFF0u + i);
        CHECK(observer.direction == YAW_RATE_DIRECTION_SAME);
        control_yaw_rate_observe(&observer, 100.0f, 100.0f, 1000);
        CHECK(observer.direction == YAW_RATE_DIRECTION_UNKNOWN); /* Debug/scheduling gap is not a rate. */
        control_yaw_rate_observe(&observer, invalid.value, 100, 1001);
        CHECK(!observer.initialized && observer.direction == YAW_RATE_DIRECTION_UNKNOWN);
        control_yaw_rate_reset(&observer);
        for (i = 0; i <= 20; ++i)
            control_yaw_rate_observe(&observer, 0, 0.037f, 1200u + i);
        CHECK(observer.direction == YAW_RATE_DIRECTION_UNKNOWN); /* Stationary noise does not choose a sign. */
        control_yaw_rate_reset(&observer);
        for (i = 0; i <= 20; ++i)
        {
            float rate = i < 10u ? 100.0f : (i == 10u ? 0.0f : -100.0f);
            float angle = (float)(i <= 10u ? i : 20u - i) * 0.1f;
            control_yaw_rate_observe(&observer, angle, rate, 1300u + i);
        }
        CHECK(observer.direction == YAW_RATE_DIRECTION_UNKNOWN); /* Net-zero reversal window. */
        SMC_Init(&provisional);
        SMC_SetParam_Exponent(&provisional, YAW_SMC_J, YAW_SMC_K, YAW_SMC_C, YAW_SMC_EPSILON,
                              2.0f, 16384.0f, 0.0f);
        SMC_ErrorUpdate(&provisional, 0, 5, 0);
        CHECK(SMC_Calculate(&provisional) < -3800 && provisional.u > -3900);
        SMC_Clear(&provisional);
        SMC_ErrorUpdate(&provisional, 0, 0, 20);
        CHECK(SMC_Calculate(&provisional) < -1800 && provisional.u > -2100);
    }
    {
        control_yaw_guard_t guard;
        union { uint32_t bits; float value; } invalid;
        invalid.bits = 0x7FC00000u;
        CHECK(control_slew(-4000, 4000, 120) > 0); /* Regression: previous torque would persist. */
        CHECK(control_yaw_command(-16384) == -YAW_OUTPUT_LIMIT);
        CHECK(control_yaw_command(16384) == YAW_OUTPUT_LIMIT);
        CHECK(control_yaw_command(12000) == 12000); /* Old 4000 commissioning cap is gone. */
        CHECK(control_yaw_command(50000) == 16384);
        CHECK(control_yaw_command(invalid.value) == 0);
        control_yaw_guard_reset(&guard);
#if YAW_MOTION_GUARD_ENABLE
        CHECK(control_yaw_guard_update(&guard, 0, 181, 0, 0xFFFFFFF0u) == 0);
        CHECK(control_yaw_guard_update(&guard, 0, 181, 0, 33) == 0);
        CHECK(control_yaw_guard_update(&guard, 0, 181, 0, 34) == YAW_FAULT_OVERSPEED);
        CHECK(control_yaw_guard_update(&guard, 0, 0, 0, 35) == YAW_FAULT_OVERSPEED);
        control_yaw_guard_reset(&guard);
        CHECK(control_yaw_guard_update(&guard, 46, 0, 4000, 100) == 0);
        control_yaw_guard_pause(&guard); /* Loss of input cannot accumulate fault duration. */
        CHECK(control_yaw_guard_update(&guard, 46, 0, 4000, 1000) == 0);
        CHECK(control_yaw_guard_update(&guard, 0, 0, 0, 1049) == 0);
        CHECK(control_yaw_guard_update(&guard, 46, 0, 4000, 1050) == 0);
        CHECK(control_yaw_guard_update(&guard, 46, 0, 4000, 1099) == 0);
        CHECK(control_yaw_guard_update(&guard, 46, 0, 4000, 1100) == YAW_FAULT_TRACKING);
#else
        {
            uint32_t i;
            /* Crossing both old thresholds for a full second must keep control active. */
            for (i = 0; i <= 1000; ++i)
                CHECK(control_yaw_guard_update(&guard, 90, 250, 12000, i) == YAW_FAULT_NONE);
            guard.fault = YAW_FAULT_OVERSPEED;
            CHECK(control_yaw_guard_update(&guard, 0, 0, 0, 1001) == YAW_FAULT_NONE);
            guard.fault = YAW_FAULT_TRACKING;
            CHECK(control_yaw_guard_update(&guard, 0, 0, 0, 1002) == YAW_FAULT_NONE);
        }
#endif
        control_yaw_guard_reset(&guard);
        CHECK(control_yaw_guard_update(&guard, 0, 0, invalid.value, 1101) == YAW_FAULT_INVALID);
        CHECK(control_yaw_guard_update(&guard, 0, 0, 0, 1102) == YAW_FAULT_INVALID);
    }
#endif
    {
        control_yaw_tracker_t tracker = {0};
        union { uint32_t bits; float value; } invalid;
        invalid.bits = 0x7FC00000u;
        CHECK(control_unwrap_yaw(&tracker, invalid.value) == 0 && !tracker.initialized);
        CHECK(control_unwrap_yaw(&tracker, 179) == 179);
        CHECK(control_unwrap_yaw(&tracker, -179) == 181);
        CHECK(control_unwrap_yaw(&tracker, invalid.value) == 181);
        CHECK(control_unwrap_yaw(&tracker, 179) == 179);
    }
    CHECK(control_wrap_deg(358) == -2 && control_wrap_deg(-358) == 2);
    CHECK(control_encoder_yaw(3156, 3156, 1) == 0);
    CHECK(control_encoder_yaw(5204, 3156, 1) == 90);
    CHECK(control_encoder_yaw(3156, 5204, 1) == -90);
    CHECK(control_encoder_yaw(2, 8190, 1) < 1.0f && control_encoder_yaw(2, 8190, 1) > 0);
    CHECK(!board_motion_enabled());
    receiver_frame(364, 1024); CHECK(rc_ctrl.rc.ch[6] == 0 && key.sw == 0);
    receiver_frame(1811, 240); CHECK(rc_ctrl.rc.ch[6] == 1 && key.sw == 0 && rc_ctrl.rc.ch[8] == -1);
    receiver_frame(240, 1811); CHECK(rc_ctrl.rc.ch[6] == -1 && key.sw == 1 && rc_ctrl.rc.ch[8] == 1);
#if BOARD_ROLE == BOARD_UPPER
    CHECK(board_motion_enabled());
    CAN_cmd_pitch(500); CHECK(last_bus == &hcan1 && last_id == 0x1FE && get16(last_data + 2) == 500);
    CAN_cmd_shoot(-100, 200, 700); CHECK(last_id == 0x200 && get16(last_data + 4) == 700);
    count = tx_count; CAN_cmd_chassis(1, 2, 3, 4); CAN_cmd_yaw(-500); CHECK(tx_count == count);
    key.ch0 = 1354; key.ch1 = 800; key.sw = 1; key.pause = 1; key.btn_l = 1;
    key.key_w = 1; key.key_a = 1; key.key_shift = 1; key.key_f = 1; calibrate.flag = 1;
    test_can1.ESR = 0x12340000u; hcan1.ErrorCode = 0x10u;
    board_link_poll(); CHECK(board_armed == 1);
    CHECK(board_diag.poll_count == 1 && board_diag.chassis_tx_queued_count == 1);
    CHECK(board_diag.can1_esr == 0x12340000u && board_diag.can1_hal_error == 0x10u);
    test_can1.ESR = 0; hcan1.ErrorCode = 0;
    CHECK(input_tx_count == 1 && yaw_tx_count == 1);
    CHECK(get16(sent_input) == 1354 && get16(sent_input + 2) == 800);
    CHECK(((uint16_t)get16(sent_input + 4) & 0x0FFFu) == 0x0F5Du);
    CHECK(get16(sent_yaw) == -500 && sent_yaw[2] == 1);
    tick = 1001; board_link_poll(); CHECK(input_tx_count == 1 && yaw_tx_count == 1);
    tick = 1002; board_link_poll(); CHECK(input_tx_count == 1 && yaw_tx_count == 2);
    tick = 1010; board_link_poll(); CHECK(input_tx_count == 2);
    tick = 1022; board_link_poll(); CHECK(sent_yaw[2] == 0 && board_motion_enabled());
    feedback(&hcan1, BOARD_YAW_FEEDBACK_ID, 789);
    CHECK(get_yaw_gimbal_motor_measure_point()->speed_rpm == 789);
    feedback(&hcan1, 0x201, -123); CHECK(get_friction_motor_measure_point(0)->speed_rpm == -123);
    tick = 1101; CHECK(!board_motion_enabled());
    CAN_cmd_pitch(500); CHECK(get16(last_data + 2) == 0);
    CAN_cmd_shoot(100, 200, 700); CHECK(get16(last_data + 4) == 0);
    board_link_poll(); CHECK(!((uint16_t)get16(sent_input + 4) & 0x0800u));
    count = board_tx_fail_count; mailbox_free = 0;
    tick = 1112; board_link_poll(); CHECK(board_tx_fail_count > count);
    CHECK(board_diag.chassis_tx_queued_count == input_tx_count);
    mailbox_free = 3; tick = 1113; board_link_poll(); CHECK(input_tx_count > 2);
    {
        uint32_t saved_input_count = input_tx_count;
        /* Both are due, only one mailbox remains. Chassis must take it. */
        test_mailbox_consumes = 1; test_can1.TSR = CAN_TSR_TME0;
        tick = 1123; board_note_input(); board_link_poll();
        CHECK(input_tx_count == saved_input_count + 1);
        test_mailbox_consumes = 0; test_can1.TSR = CAN_TSR_TME;
    }
    {
        uint8_t d[8] = {3, 1, 0, 0x4C, 0x4F, 0x57, 0, 0};
        d[7] = checksum(d); board_link_receive(BOARD_STATUS_ID, d);
        CHECK(board_lower_online() && board_remote_motor_mask == 0);
    }
    /* Separate motor feedback freshness from CAN1 board-link traffic. */
    tick = 2000;
    feedback(&hcan1, 0x202, 0);
    count = board_diag.motor_can1_rx_count[1];
    CHECK(count != 0 && board_diag.motor_can1_last_tick_ms[1] == 2000);
    tick = 2010; board_link_poll();
    CHECK(board_diag.motor_can1_age_ms[1] == 10);
    CHECK(board_diag.motor_can1_age_ms[3] == UINT32_MAX);
    count = board_diag.shooter_tx_queued_count;
    CAN_cmd_shoot(0, 0, 0);
    CHECK(board_diag.shooter_tx_queued_count == count + 1);
    count = board_diag.shooter_tx_fail_count;
    mailbox_free = 0; CAN_cmd_shoot(0, 0, 0); mailbox_free = 3;
    CHECK(board_diag.shooter_tx_fail_count == count + 1);
    {
        uint8_t i;
        uint8_t payload[8] = {0};
        uint32_t expired, queued, aborts;
        /* Actual deadline code: every mailbox, timer wrap, asynchronous abort,
         * fresh-only resumption, cancellation failure and hardware completion. */
        test_mailbox_consumes = 1;
        for (i = 0; i < 3; ++i)
        {
            memset(can1_tx_lifetime, 0, sizeof(can1_tx_lifetime));
            test_can1.TSR = CAN_TSR_TME0 << i;
            tick = 0xFFFFFFF8u;
            CHECK(transmit(&hcan1, BOARD_CHASSIS_INPUT_ID, payload));
            CHECK(can1_tx_lifetime[i].active && can1_tx_lifetime[i].tick == tick);
            expired = board_diag.can1_tx_expired_count;
            aborts = abort_count[0];
            tick += BOARD_CAN1_TX_LIFETIME_MS - 1;
            CHECK(expire_can1_tx_locked() && abort_count[0] == aborts);
            ++tick;
            CHECK(!expire_can1_tx_locked() && last_abort_mask == (1u << i));
            CHECK(board_diag.can1_tx_expired_count == expired + 1);
            queued = tx_count;
            CHECK(!transmit(&hcan1, 0x1FE, payload) && tx_count == queued);
            CHECK(abort_count[0] == aborts + 1); /* No repeated abort while pending. */
            test_can1.TSR = CAN_TSR_TME;
            CHECK(expire_can1_tx_locked() && !can1_tx_lifetime[i].active);
            payload[0] = i + 1;
            CHECK(transmit(&hcan1, 0x1FE, payload) && last_data[0] == i + 1);
        }
        memset(can1_tx_lifetime, 0, sizeof(can1_tx_lifetime));
        test_can1.TSR = CAN_TSR_TME0;
        tick = 100;
        CHECK(transmit(&hcan1, BOARD_CHASSIS_INPUT_ID, payload));
        abort_status = 1; tick += BOARD_CAN1_TX_LIFETIME_MS;
        aborts = board_diag.can1_tx_abort_fail_count;
        CHECK(!expire_can1_tx_locked());
        CHECK(board_diag.can1_tx_abort_fail_count == aborts + 1 && !can1_tx_lifetime[0].aborting);
        abort_status = HAL_OK;
        CHECK(!expire_can1_tx_locked() && can1_tx_lifetime[0].aborting);
        test_can1.TSR = CAN_TSR_TME;
        CHECK(expire_can1_tx_locked());
        CHECK(transmit(&hcan1, BOARD_CHASSIS_INPUT_ID, payload));
        expired = board_diag.can1_tx_expired_count;
        test_can1.TSR = CAN_TSR_TME; tick += 100;
        CHECK(expire_can1_tx_locked() && board_diag.can1_tx_expired_count == expired);
        test_mailbox_consumes = 0; tick = 2010;
    }
    tick = 3000; key.sw = 1; key.ch2 = key.ch3 = 1024;
    key.mouse_x = key.mouse_y = 0; key.mouse_btn_r = 0;
    calibrate.flag = 0; mahony.YAW = 179; mahony.PITCH = 0;
    mahony.g_smc[0] = mahony.g_smc[1] = mahony.g_smc[2] = 0;
    key.sw = 0; rc_ctrl.rc.ch[8] = 0;
    board_note_input(); test_shooter_loop = 1;
    SHOOTER_TASK(0);
#elif BOARD_ROLE == BOARD_LOWER
    CHECK(!board_motion_enabled());
    CHECK(!board_prepare_chassis_input() && lower_diag.input_reason == 1);
    CHECK(lower_diag.input_age_ms == UINT32_MAX && lower_diag.input_loss_count == 0);
    CAN_cmd_pitch(1); CAN_cmd_shoot(1, 2, 3); CHECK(tx_count == 0);
    input_frame(1, 0x0800, 1);
    CHECK(board_diag.chassis_reject_reason == BOARD_CHASSIS_BAD_CHECKSUM && board_chassis_rx_count == 0);
    input_frame_xy(1, 0x0800, 0, 2048, 1024);
    CHECK(board_diag.chassis_reject_reason == BOARD_CHASSIS_BAD_CHANNEL && board_chassis_rx_count == 0);
    {
        uint8_t d[8] = {0};
        put16(d, 1024); put16(d + 2, 1024); put16(d + 4, 0x2800);
        d[6] = 1; d[7] = checksum(d);
        rx_header.DLC = 8; memcpy(rx_data, d, 8); rx_pending = 1;
        HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
        CHECK(board_diag.chassis_frame_count == 3 && board_diag.chassis_rx_version == 2);
        CHECK(board_diag.chassis_reject_reason == BOARD_CHASSIS_BAD_VERSION && !board_motion_enabled());
    }
    input_frame(1, 0x0F5D, 0); CHECK(board_motion_enabled() && !board_yaw_enabled());
    CHECK(board_diag.chassis_reject_reason == BOARD_CHASSIS_ACCEPTED && board_diag.chassis_rx_version == 3);
    CHECK(board_prepare_chassis_input());
    CHECK(lower_diag.input_reason == 0 && lower_diag.input_age_ms == 0);
    CHECK(lower_diag.ch0 == 1354 && lower_diag.ch1 == 1024 && lower_diag.mode == 1);
    CHECK(key.ch0 == 1354 && key.ch1 == 1024 && key.sw == 1 && key.pause == 1 && key.btn_l == 1);
    CHECK(key.key_w == 1 && key.key_s == 0 && key.key_a == 1 && key.key_d == 0 && key.key_shift == 1 && key.key_f == 1);
    CHECK(board_take_chassis_reset()); CHECK(!board_take_chassis_reset());
    CAN_cmd_chassis(100, -200, 300, -400);
    CHECK(last_bus == &hcan2 && last_id == 0x200 && get16(last_data + 6) == -400);
    CHECK(lower_diag.chassis_command[0] == 100 && lower_diag.chassis_command[3] == -400);
    CHECK(lower_diag.can2_tx_queued_count == 1 && lower_diag.can2_tx_fail_count == 0);
    mailbox_free = 0;
    CAN_cmd_chassis(100, -200, 300, -400);
    CHECK(lower_diag.can2_tx_queued_count == 1 && lower_diag.can2_tx_fail_count == 1);
    CHECK(lower_diag.chassis_command[0] == 100 && test_can2.ESR == 0); /* ESR=0 does not prove TX. */
    mailbox_free = 3;
    yaw_frame(1, 1, 0); CHECK(board_yaw_enabled());
    CAN_cmd_yaw(-500); CHECK(last_id == 0x1FE && get16(last_data) == -500);
    CHECK(lower_diag.yaw_received == -500 && lower_diag.yaw_command == -500);
    test_can2.MCR = 0x10; test_can2.MSR = 0;
    test_can2.TSR = 0x1C000000u; hcan2.State = 2; hcan2.ErrorCode = 0x20;
    count = chassis_tx_count; board_link_poll(); CHECK(chassis_tx_count == count);
    CHECK(board_diag.poll_count == 1 && board_diag.chassis_tx_queued_count == 0);
    CHECK(lower_diag.motion_enabled == 1 && lower_diag.yaw_enabled == 1);
    CHECK(lower_diag.can2_state == 2 && lower_diag.can2_error == 0x20);
    CHECK(lower_diag.can2_mcr == 0x10 && lower_diag.can2_msr == 0);
    CHECK(lower_diag.can2_tsr == 0x1C000000u && lower_diag.can2_esr == 0);
    hcan2.ErrorCode = 0;
    count = board_chassis_rx_count; input_frame(2, 0x0800, 1); CHECK(board_chassis_rx_count == count);
    input_frame(2, 0, 0); CHECK(!board_motion_enabled() && board_yaw_enabled());
    count = lower_diag.send_gate_drop_count;
    CAN_cmd_chassis(1, 2, 3, 4); CHECK(get16(last_data) == 0);
    CHECK(lower_diag.send_gate_drop_count == count + 1);
    input_frame(3, 0x0800, 0); yaw_frame(2, 0, 0); CHECK(board_motion_enabled() && !board_yaw_enabled());
    CAN_cmd_yaw(-500); CHECK(get16(last_data) == 0);
    CHECK(lower_diag.yaw_received == -500 && lower_diag.yaw_command == 0);
    count = board_chassis_rx_count; tick = 1050; input_frame(3, 0x0800, 0); CHECK(board_chassis_rx_count == count);
    CHECK(board_diag.chassis_reject_reason == BOARD_CHASSIS_OLD_SEQUENCE);
    tick = 1101; CHECK(!board_motion_enabled()); /* Duplicate cannot refresh. */
    tick = 0xFFFFFFF0u; input_frame(4, 0x0800, 0); yaw_frame(3, 1, 0);
    tick = 20; CHECK(board_motion_enabled() && board_yaw_enabled());
    tick = 200; CHECK(!board_motion_enabled() && !board_yaw_enabled());
    /* LOWER-specific control regression, using actual PID and task helpers. */
    {
        PID_t pid;
        float out;
        uint32_t i;
        PID_Init(&pid, 0.95f, 0.1f, 0, 0, 10, 10, 0);
        PID_Set_Values(&pid, 100, 0);
        for (i = 0; i < 2000; ++i) chassis_speed_pid(&pid, &out);
        CHECK(out == 10 && pid.Integral_Error == 0);
        PID_Set_Values(&pid, -100, 0);
        for (i = 0; i < 2000; ++i) chassis_speed_pid(&pid, &out);
        CHECK(out == -10 && pid.Integral_Error == 0);
        PID_Clear_Error(&pid);
        PID_Set_Values(&pid, 5, 0); chassis_speed_pid(&pid, &out);
        CHECK(out > 4.75f && pid.Integral_Error > 0); /* I still works below saturation. */
        pid.Integral_Error = -100; pid.Pre_Target = 100;
        PID_Set_Values(&pid, 100, 80); chassis_speed_pid(&pid, &out);
        CHECK(pid.Integral_Error > -100); /* Opposing integral unwinds. */
        pid.Integral_Error = 80; pid.Pre_Target = 100;
        PID_Set_Values(&pid, 0, 5); chassis_speed_pid(&pid, &out);
        CHECK(out < 0 && control_abs(pid.Integral_Error) < 0.01f);
        pid.Integral_Error = 80; pid.Pre_Target = 100;
        PID_Set_Values(&pid, -1, 0); chassis_speed_pid(&pid, &out);
        CHECK(out < 0 && control_abs(pid.Integral_Error) < 0.01f);

        PID_Init(&pid, FOLLOW_KP, FOLLOW_KI, 0, 0, 1, FOLLOW_OUT_MAX, FOLLOW_DEAD_ZONE);
        for (i = 0; i < 10000; ++i) out = chassis_follow_control(&pid, 30, 1);
        CHECK(out > 0);
        CHECK(chassis_follow_control(&pid, 0, 1) == 0 && pid.Integral_Error == 0);
        (void)chassis_follow_control(&pid, 30, 1);
        CHECK(chassis_follow_control(&pid, 30, 0) == 0 && pid.Integral_Error == 0);
        (void)chassis_follow_control(&pid, 30, 1);
        CHECK(chassis_follow_control(&pid, -3, 1) < 0 && pid.Integral_Error < 0);

        CHECK(chassis_rc_axis(1024) == 0 && chassis_rc_axis(1044) == 0 && chassis_rc_axis(1004) == 0);
        CHECK(chassis_rc_axis(1045) > 0 && chassis_rc_axis(1045) < 0.002f);
        CHECK(chassis_rc_axis(1003) == -chassis_rc_axis(1045));
        CHECK(chassis_rc_axis(1807) == CHASSIS_RC_MAX_SPEED_MPS);
        CHECK(chassis_rc_axis(2047) == CHASSIS_RC_MAX_SPEED_MPS);
        CHECK(chassis_rc_axis(0) == -CHASSIS_RC_MAX_SPEED_MPS);
        CHECK(board_motor_feedback_age(5) == UINT32_MAX);
        CHECK(board_motor_feedback_age(4) == UINT32_MAX);
        gimbal_vx_applied = gimbal_vy_applied = chassis_omega_applied = 0;
        for (i = 0; i < 4; ++i) chassis_motion.wheel_now_omega[i] = 0;
        CHECK(chassis_can_release(1));
        chassis_motion.wheel_now_omega[3] = 2;
        CHECK(chassis_can_release(1)); /* Once released, feedback alone must not re-arm braking. */
        CHECK(!chassis_can_release(0));
        CHECK(!chassis_can_release(1)); /* Must inspect all four wheels. */
        {
            uint32_t saved_tick = tick;
            uint32_t saved_timeouts = lower_diag.brake_timeout_count;
            tick += CHASSIS_BRAKE_TIMEOUT_MS - 1;
            CHECK(!chassis_can_release(1));
            ++tick;
            CHECK(chassis_can_release(1));
            CHECK(lower_diag.brake_timeout_count == saved_timeouts + 1);
            CHECK(chassis_can_release(1));
            CHECK(lower_diag.brake_timeout_count == saved_timeouts + 1);
            tick = saved_tick;
        }
        chassis_motion.wheel_now_omega[3] = 0;
        CHECK(!chassis_can_release(0));
        gimbal_vx_applied = 0.001f;
        CHECK(!chassis_can_release(1)); /* Do not truncate the last ramp steps. */
        gimbal_vx_applied = 0;
        CHECK(!chassis_can_release(0));
    }
    /* A constant gun-relative translation must rotate with the coordinate
     * basis immediately; body-coordinate slew must not drag its direction. */
    {
        uint8_t direction;
        float angles[5] = {0, 90, 180, -90, 0};
        float expected_x[5] = {0.5f, 0, -0.5f, 0, 0.5f};
        float expected_y[5] = {0, 0.5f, 0, -0.5f, 0};
        gimbal_vx_applied = 0.5f; gimbal_vy_applied = 0;
        chassis_omega_applied = 2.0f * PI;
        chassis_motion.gimbal_target_vx = 0.5f;
        chassis_motion.gimbal_target_vy = 0;
        for (direction = 0; direction < 5; ++direction)
        {
            float turn, wx, wy, wr;
            chassis_motion.chassis_target_omega = 2.0f * PI;
            gimbal_coordinate_resolution(angles[direction], &chassis_motion);
            CHECK(control_abs(chassis_motion.chassis_target_vx - expected_x[direction]) < 0.0001f);
            CHECK(control_abs(chassis_motion.chassis_target_vy - expected_y[direction]) < 0.0001f);
            /* Forward kinematics recovers translation and spin simultaneously. */
            turn = WHEEL_RADIUS / GEAR_RATIO;
            wx = turn * (chassis_motion.wheel_target_omega[0] + chassis_motion.wheel_target_omega[1] -
                         chassis_motion.wheel_target_omega[2] - chassis_motion.wheel_target_omega[3]) / (4 * 0.707f);
            wy = turn * (-chassis_motion.wheel_target_omega[0] + chassis_motion.wheel_target_omega[1] +
                         chassis_motion.wheel_target_omega[2] - chassis_motion.wheel_target_omega[3]) / (4 * 0.707f);
            wr = -turn * (chassis_motion.wheel_target_omega[0] + chassis_motion.wheel_target_omega[1] +
                          chassis_motion.wheel_target_omega[2] + chassis_motion.wheel_target_omega[3]) / (4 * WHEEL_TO_CORE_DISTANCE);
            CHECK(control_abs(wx - expected_x[direction]) < 0.0001f);
            CHECK(control_abs(wy - expected_y[direction]) < 0.0001f);
            CHECK(control_abs(wr - 2.0f * PI) < 0.0001f);
        }
        gimbal_vx_applied = gimbal_vy_applied = chassis_omega_applied = 0;
    }
    /* Execute the actual unchanged-gain chassis PID with local CAN2 updates. */
    {
        uint16_t i;
        float angles[4] = {0, 90, -90, 180};
        float expected_x[4] = {0.5f, 0, 0, -0.5f};
        float expected_y[4] = {0, 0.5f, -0.5f, 0};
        uint8_t direction;
        for (direction = 0; direction < 4; ++direction)
        {
            chassis_motion.gimbal_target_vx = 0.5f; chassis_motion.gimbal_target_vy = 0;
            chassis_motion.chassis_target_omega = 0;
            gimbal_vx_applied = gimbal_vy_applied = chassis_omega_applied = 0;
            for (i = 0; i < 400; ++i) gimbal_coordinate_resolution(angles[direction], &chassis_motion);
            CHECK(control_abs(chassis_motion.chassis_target_vx - expected_x[direction]) < 0.0001f);
            CHECK(control_abs(chassis_motion.chassis_target_vy - expected_y[direction]) < 0.0001f);
        }
        gimbal_vx_applied = gimbal_vy_applied = chassis_omega_applied = 0;
    }
    tick = 2000; input_frame_xy(40, 0x0800, 0, 1024, 1354);
    feedback(&hcan2, 0x205, 0);
    CHECK(lower_diag.can2_rx_count == 1 && lower_diag.motor_rx_count[4] == 1);
    rx_header.DLC = 7; rx_pending = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    CHECK(lower_diag.can2_invalid_frame_count == 1 && lower_diag.motor_rx_count[4] == 1);
    rx_status = 1; rx_pending = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2); rx_status = HAL_OK;
    CHECK(lower_diag.can2_rx_error_count == 1 && lower_diag.can2_rx_count == 2);
    feedback(&hcan2, 0x201, 0); feedback(&hcan2, 0x202, 0);
    feedback(&hcan2, 0x203, 0); feedback(&hcan2, 0x204, 0); feedback(&hcan2, 0x205, 0);
    local_tx_start = chassis_tx_count; test_local_loop = 1;
    local_input_loss_start = lower_diag.input_loss_count;
    CHASSIS_TASK(0);
#else
    CAN_cmd_pitch(500); CHECK(last_bus == &hcan1 && get16(last_data + 2) == 500);
    CAN_cmd_shoot(-100, 200, 700); CHECK(last_bus == &hcan1 && get16(last_data + 4) == 700);
    CAN_cmd_chassis(1, 2, 3, 4); CHECK(last_bus == &hcan2 && get16(last_data + 6) == 4);
    CAN_cmd_yaw(500); CHECK(last_bus == &hcan2 && get16(last_data) == 500);
    feedback(&hcan1, 0x203, -123); CHECK(get_trigger_motor_measure_point()->speed_rpm == -123);
    feedback(&hcan2, 0x201, 456); CHECK(get_chassis_motor_measure_point(0)->speed_rpm == 456);
#endif
    test_done();
}
