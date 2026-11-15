#include "board_link.h"
#include "drv_CAN_receive.h"
#include "cmsis_os.h"
#include <string.h>

volatile uint8_t board_armed;
volatile uint32_t board_rx_count;
volatile uint32_t board_tx_fail_count;
volatile uint8_t board_remote_motor_mask;
volatile uint32_t board_chassis_rx_count;
volatile uint32_t board_yaw_rx_count;
volatile board_link_diag_t board_diag = {BOARD_ROLE, BOARD_PROTOCOL_VERSION};
#if BOARD_ROLE == BOARD_LOWER
volatile board_lower_diag_t lower_diag;
#endif

static volatile uint32_t input_tick, chassis_tick, yaw_tick, status_tick, yaw_cache_tick;
static volatile uint8_t input_seen, chassis_seen, chassis_enable, yaw_seen, yaw_enable, status_seen, yaw_cache_seen;
static int16_t yaw_command;
static VTM_Data_t chassis_input;
static volatile uint8_t chassis_reset_pending;
static uint8_t feedback_data[5][8], feedback_seen[5];
static uint32_t feedback_tick[5];
static uint8_t chassis_sequence_seen, last_chassis_sequence;
static uint8_t yaw_sequence_seen, last_yaw_sequence;
static uint8_t status_sequence_seen, last_status_sequence;
#if BOARD_ROLE == BOARD_UPPER
typedef struct {
    uint32_t tick;
    uint8_t active, aborting;
} board_tx_lifetime_t;
static board_tx_lifetime_t can1_tx_lifetime[3];
#endif

#if BOARD_ROLE != BOARD_SINGLE
typedef struct {
    volatile uint8_t stage;
    uint8_t abort_pending;
} board_can_recovery_t;
static board_can_recovery_t can_recovery[2];

/* RM0090: with ABOM disabled, Bus-Off recovery requires entering/leaving
 * initialization and then 128 occurrences of 11 recessive bits. Poll the
 * hardware acknowledgments; never block the control tasks waiting for them.
 * Do not reset CAN1: its filter block is also used by CAN2. */
static void service_can(CAN_HandleTypeDef *can, board_can_recovery_t *recovery,
                        volatile board_can_diag_t *diag)
{
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    uint32_t esr = can->Instance->ESR;
    if (!recovery->stage)
    {
        uint8_t busoff = (esr & CAN_ESR_BOFF) != 0u;
        uint8_t start_failed =
            (can->State == HAL_CAN_STATE_ERROR && (can->ErrorCode & HAL_CAN_ERROR_TIMEOUT)) ||
            (can->State == HAL_CAN_STATE_READY && (can->Instance->MSR & CAN_MSR_INAK));
        if (!busoff && !start_failed) return;
        recovery->stage = 1; recovery->abort_pending = 1;
        diag->stage = 1; diag->last_error_esr = esr; diag->last_hal_error = can->ErrorCode;
        if (busoff) ++diag->busoff_count;
        else ++diag->start_retry_count;
        /* HAL init/start already configured the peripheral. Restore the HAL
         * state only for a timed-out start so cancellation can be requested. */
        if (can->State == HAL_CAN_STATE_ERROR) can->State = HAL_CAN_STATE_READY;
        can->Instance->MCR |= CAN_MCR_INRQ;
    }
    if (recovery->stage == 1)
    {
        if (recovery->abort_pending)
        {
            if (HAL_CAN_AbortTxRequest(can, mailboxes) != HAL_OK)
            { ++diag->abort_fail_count; return; }
            recovery->abort_pending = 0;
        }
        if ((can->Instance->MSR & CAN_MSR_INAK) == 0u) return;
        /* Mirrors HAL_CAN_Start's state transition, without its blocking wait. */
        can->State = HAL_CAN_STATE_LISTENING;
        can->Instance->MCR &= ~(CAN_MCR_INRQ | CAN_MCR_SLEEP);
        recovery->stage = 2; diag->stage = 2;
        return;
    }
    if ((can->Instance->MSR & CAN_MSR_INAK) || (can->Instance->ESR & CAN_ESR_BOFF)) return;
    /* An abort is asynchronous. Do not allow old requests to survive recovery. */
    if ((can->Instance->TSR & CAN_TSR_TME) != CAN_TSR_TME) return;
    if (HAL_CAN_ActivateNotification(can, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    { ++diag->notify_fail_count; return; }
    can->ErrorCode = HAL_CAN_ERROR_NONE;
    ++diag->recovered_count;
    recovery->stage = 0; diag->stage = 0;
}

static uint8_t can_ready_to_send(const CAN_HandleTypeDef *can)
{
    uint8_t index = can->Instance == CAN1 ? 0u : 1u;
    return !can_recovery[index].stage && can->State == HAL_CAN_STATE_LISTENING &&
           !(can->Instance->ESR & CAN_ESR_BOFF) && !(can->Instance->MSR & CAN_MSR_INAK) &&
           !(can->Instance->MCR & CAN_MCR_INRQ);
}
#endif

static uint32_t lock_irq(void)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq(); return mask;
}
static void unlock_irq(uint32_t mask) { __set_PRIMASK(mask); }
static uint8_t fresh(uint32_t tick, uint32_t limit)
{
    return (uint32_t)(HAL_GetTick() - tick) <= limit;
}
static void put16(uint8_t *p, int16_t value)
{
    p[0] = (uint8_t)((uint16_t)value >> 8); p[1] = (uint8_t)value;
}
static int16_t get16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static uint8_t checksum(const uint8_t data[8])
{
    uint8_t value = 0xA5u, i;
    for (i = 0; i < 7; ++i) value ^= data[i];
    return value;
}
static uint8_t advanced(uint8_t seq, uint8_t previous)
{
    uint8_t delta = (uint8_t)(seq - previous);
    return delta != 0u && delta < 128u;
}
#if BOARD_ROLE == BOARD_UPPER
/* Called with IRQs locked. An abort request does not make a mailbox empty;
 * wait for TME before allowing any new CAN1 traffic to overtake an old frame. */
static uint8_t expire_can1_tx_locked(void)
{
    uint8_t i, ready = 1;
    uint32_t now = HAL_GetTick();
    if (can_recovery[0].stage) return 0;
    for (i = 0; i < 3; ++i)
    {
        board_tx_lifetime_t *slot = &can1_tx_lifetime[i];
        if (!slot->active) continue;
        if (hcan1.Instance->TSR & (CAN_TSR_TME0 << i))
        {
            slot->active = slot->aborting = 0;
            continue;
        }
        if (slot->aborting) { ready = 0; continue; }
        if ((uint32_t)(now - slot->tick) < BOARD_CAN1_TX_LIFETIME_MS) continue;
        ready = 0;
        if (HAL_CAN_AbortTxRequest(&hcan1, 1u << i) == HAL_OK)
        {
            slot->aborting = 1;
            ++board_diag.can1_tx_expired_count;
        }
        else ++board_diag.can1_tx_abort_fail_count;
    }
    return ready;
}
#endif
static uint8_t transmit(CAN_HandleTypeDef *can, uint32_t id, uint8_t data[8])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox, mask;
    uint8_t ok;
    header.StdId = id; header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA; header.DLC = 8;
    mask = lock_irq();
    ok =
#if BOARD_ROLE == BOARD_UPPER
         (can->Instance != CAN1 || expire_can1_tx_locked()) &&
#endif
#if BOARD_ROLE != BOARD_SINGLE
         can_ready_to_send(can) &&
#endif
         HAL_CAN_GetTxMailboxesFreeLevel(can) != 0u &&
         HAL_CAN_AddTxMessage(can, &header, data, &mailbox) == HAL_OK;
    if (!ok) ++board_tx_fail_count;
#if BOARD_ROLE == BOARD_UPPER
    if (ok && can->Instance == CAN1)
    {
        uint8_t i;
        for (i = 0; i < 3; ++i)
            if (mailbox == (1u << i))
            {
                can1_tx_lifetime[i].tick = HAL_GetTick();
                can1_tx_lifetime[i].active = 1;
                can1_tx_lifetime[i].aborting = 0;
            }
    }
    if (!ok && can->Instance == CAN1 && id == BOARD_CHASSIS_INPUT_ID)
        ++board_diag.chassis_tx_fail_count;
    if (can->Instance == CAN1 && id == CAN_CHASSIS_ALL_ID)
    {
        if (ok) ++board_diag.shooter_tx_queued_count;
        else ++board_diag.shooter_tx_fail_count;
    }
#endif
#if BOARD_ROLE == BOARD_LOWER
    if (can->Instance == CAN2)
    {
        if (ok) ++lower_diag.can2_tx_queued_count;
        else ++lower_diag.can2_tx_fail_count;
    }
#endif
    unlock_irq(mask);
    return ok;
}
void board_send_motor(CAN_HandleTypeDef *can, uint32_t id, int16_t a,
                      int16_t b, int16_t c, int16_t d)
{
    uint8_t data[8];
#if BOARD_ROLE == BOARD_LOWER
    if (can->Instance == CAN2)
    {
        if (id == CAN_CHASSIS_ALL_ID)
        {
            lower_diag.chassis_command[0] = a; lower_diag.chassis_command[1] = b;
            lower_diag.chassis_command[2] = c; lower_diag.chassis_command[3] = d;
        }
        else if (id == CAN_GIMBAL_ALL_ID) lower_diag.yaw_command = a;
    }
#endif
    put16(data, a); put16(data + 2, b); put16(data + 4, c); put16(data + 6, d);
    transmit(can, id, data);
}
void board_note_input(void)
{
    /* Original receiver/VTM has already updated key. Do not reinterpret it. */
    input_tick = HAL_GetTick(); input_seen = 1; board_armed = 1;
}
uint8_t board_motion_enabled(void)
{
#if BOARD_ROLE == BOARD_UPPER
    return input_seen && fresh(input_tick, BOARD_RC_TIMEOUT_MS);
#else
    return chassis_seen && chassis_enable && fresh(chassis_tick, BOARD_LINK_TIMEOUT_MS);
#endif
}
uint8_t board_yaw_enabled(void)
{
#if BOARD_ROLE == BOARD_UPPER
    return board_motion_enabled() && yaw_cache_seen &&
           fresh(yaw_cache_tick, BOARD_YAW_CACHE_TIMEOUT_MS);
#else
    return yaw_seen && yaw_enable && fresh(yaw_tick, BOARD_LINK_TIMEOUT_MS);
#endif
}
uint8_t board_lower_online(void)
{
    return status_seen && fresh(status_tick, BOARD_LINK_TIMEOUT_MS);
}
void board_set_yaw_command(int16_t current)
{
    yaw_command = current; yaw_cache_tick = HAL_GetTick(); yaw_cache_seen = 1;
}
void board_capture_feedback(uint8_t index, const uint8_t data[8])
{
    if (index < 5u)
    {
#if BOARD_ROLE == BOARD_LOWER
        uint32_t gap = (uint32_t)(HAL_GetTick() - feedback_tick[index]);
        if (feedback_seen[index] && gap > lower_diag.feedback_max_gap_ms[index])
            lower_diag.feedback_max_gap_ms[index] = gap;
        ++lower_diag.motor_rx_count[index];
#endif
        memcpy(feedback_data[index], data, 8);
        feedback_tick[index] = HAL_GetTick(); feedback_seen[index] = 1;
    }
}
uint32_t board_motor_feedback_age(uint8_t index)
{
    uint32_t age = UINT32_MAX;
    uint32_t mask = lock_irq();
    if (index < 5u && feedback_seen[index])
        age = (uint32_t)(HAL_GetTick() - feedback_tick[index]);
    unlock_irq(mask);
    return age;
}
uint8_t board_prepare_chassis_input(void)
{
#if BOARD_ROLE == BOARD_LOWER
    uint8_t ok;
    static uint8_t was_enabled;
    uint32_t mask = lock_irq();
    ok = board_motion_enabled();
    lower_diag.input_age_ms = chassis_seen ? (uint32_t)(HAL_GetTick() - chassis_tick) : UINT32_MAX;
    lower_diag.input_reason = !chassis_seen ? 1u :
        (lower_diag.input_age_ms > BOARD_LINK_TIMEOUT_MS ? 3u : (!chassis_enable ? 2u : 0u));
    lower_diag.motion_enabled = ok;
    if (was_enabled && !ok)
    {
        ++lower_diag.input_loss_count;
        lower_diag.last_input_loss_reason = lower_diag.input_reason;
        lower_diag.last_input_loss_tick_ms = HAL_GetTick();
        lower_diag.last_input_loss_age_ms = lower_diag.input_age_ms;
    }
    was_enabled = ok;
    if (ok)
    {
        key = chassis_input; /* One coherent snapshot per local 1ms iteration. */
        lower_diag.ch0 = key.ch0; lower_diag.ch1 = key.ch1; lower_diag.mode = key.sw;
    }
    unlock_irq(mask);
    return ok;
#else
    return 1;
#endif
}
uint8_t board_take_chassis_reset(void)
{
    uint8_t reset;
    uint32_t mask = lock_irq();
    reset = chassis_reset_pending; chassis_reset_pending = 0;
    unlock_irq(mask);
    return reset;
}
uint8_t board_link_receive(uint32_t id, const uint8_t data[8])
{
#if BOARD_ROLE == BOARD_LOWER
    if (id == BOARD_CHASSIS_INPUT_ID)
    {
        uint16_t flags = (uint16_t)get16(data + 4);
        ++board_diag.chassis_frame_count;
        board_diag.chassis_rx_version = flags >> 12;
        if ((flags >> 12) != BOARD_PROTOCOL_VERSION)
        { board_diag.chassis_reject_reason = BOARD_CHASSIS_BAD_VERSION; return 1; }
        if (data[7] != checksum(data))
        { board_diag.chassis_reject_reason = BOARD_CHASSIS_BAD_CHECKSUM; return 1; }
        if ((uint16_t)get16(data) > 2047u || (uint16_t)get16(data + 2) > 2047u)
        { board_diag.chassis_reject_reason = BOARD_CHASSIS_BAD_CHANNEL; return 1; }
        if (!(flags & 0x0800u)) chassis_enable = 0;
        if (chassis_sequence_seen && fresh(chassis_tick, BOARD_LINK_TIMEOUT_MS) &&
            !advanced(data[6], last_chassis_sequence))
        { board_diag.chassis_reject_reason = BOARD_CHASSIS_OLD_SEQUENCE; return 1; }
        chassis_input.ch0 = (uint16_t)get16(data);
        chassis_input.ch1 = (uint16_t)get16(data + 2);
        chassis_input.sw = flags & 3u;
        chassis_input.pause = (flags >> 2) & 1u;
        chassis_input.btn_l = (flags >> 3) & 1u;
        chassis_input.key_w = (flags >> 4) & 1u;
        chassis_input.key_s = (flags >> 5) & 1u;
        chassis_input.key_a = (flags >> 6) & 1u;
        chassis_input.key_d = (flags >> 7) & 1u;
        chassis_input.key_shift = (flags >> 8) & 1u;
        chassis_input.key_f = (flags >> 9) & 1u;
        if (flags & 0x0400u) chassis_reset_pending = 1;
        chassis_enable = (flags >> 11) & 1u;
        if (chassis_seen)
        {
            uint32_t gap = (uint32_t)(HAL_GetTick() - chassis_tick);
            if (gap > lower_diag.input_max_gap_ms) lower_diag.input_max_gap_ms = gap;
        }
        chassis_tick = HAL_GetTick(); chassis_seen = 1;
        last_chassis_sequence = data[6]; chassis_sequence_seen = 1;
        board_diag.chassis_reject_reason = BOARD_CHASSIS_ACCEPTED;
        board_armed = board_motion_enabled(); ++board_chassis_rx_count; ++board_rx_count;
        return 1;
    }
    if (id == BOARD_YAW_COMMAND_ID)
    {
        if (data[2] > 1u || data[4] != BOARD_PROTOCOL_VERSION ||
            data[5] != 0x55u || data[6] != 0xAAu || data[7] != checksum(data)) return 1;
        if (!data[2]) yaw_enable = 0;
        if (yaw_sequence_seen && fresh(yaw_tick, BOARD_LINK_TIMEOUT_MS) &&
            !advanced(data[3], last_yaw_sequence)) return 1;
        yaw_command = get16(data); yaw_enable = data[2];
        lower_diag.yaw_received = yaw_command;
        yaw_tick = HAL_GetTick(); yaw_seen = 1;
        last_yaw_sequence = data[3]; yaw_sequence_seen = 1;
        ++board_yaw_rx_count; ++board_rx_count; return 1;
    }
#else
    if (id == BOARD_STATUS_ID)
    {
        if (data[0] != BOARD_PROTOCOL_VERSION || (data[2] & 0xE0u) ||
            data[3] != 0x4Cu || data[4] != 0x4Fu || data[5] != 0x57u ||
            data[6] != 0 || data[7] != checksum(data)) return 1;
        if (status_sequence_seen && fresh(status_tick, BOARD_LINK_TIMEOUT_MS) &&
            !advanced(data[1], last_status_sequence)) return 1;
        last_status_sequence = data[1]; status_sequence_seen = 1;
        board_remote_motor_mask = data[2];
        status_tick = HAL_GetTick(); status_seen = 1; ++board_rx_count;
        return 1;
    }
#endif
    return 0;
}
void board_link_poll(void)
{
    uint32_t now = HAL_GetTick();
#if BOARD_ROLE != BOARD_SINGLE
    service_can(&hcan1, &can_recovery[0], &board_diag.can1);
    service_can(&hcan2, &can_recovery[1], &board_diag.can2);
#endif
    ++board_diag.poll_count;
    board_diag.can1_esr = hcan1.Instance->ESR;
    board_diag.can1_hal_error = hcan1.ErrorCode;
    board_armed = board_motion_enabled();
#if BOARD_ROLE == BOARD_UPPER
    {
        uint8_t i;
        static uint8_t poll_seen;
        static uint32_t previous_poll;
        uint32_t mask = lock_irq();
        uint32_t gap = (uint32_t)(now - previous_poll);
        if (poll_seen && gap > board_diag.poll_max_gap_ms) board_diag.poll_max_gap_ms = gap;
        previous_poll = now; poll_seen = 1;
        (void)expire_can1_tx_locked();
        board_diag.can1_mcr = hcan1.Instance->MCR;
        board_diag.can1_tsr = hcan1.Instance->TSR;
        for (i = 0; i < 7; ++i)
            board_diag.motor_can1_age_ms[i] = board_diag.motor_can1_rx_count[i] ?
                (uint32_t)(HAL_GetTick() - board_diag.motor_can1_last_tick_ms[i]) : UINT32_MAX;
        unlock_irq(mask);
    }
    static uint8_t yaw_started, input_started, yaw_seq, input_seq;
    static uint32_t yaw_tx_tick, input_tx_tick;
    /* A due chassis snapshot takes the last mailbox before the 2ms yaw frame.
     * Otherwise a busy CAN1 can repeatedly admit yaw and starve chassis. */
    if (!input_started || (uint32_t)(now - input_tx_tick) >= BOARD_CHASSIS_INPUT_PERIOD_MS)
    {
        VTM_Data_t snapshot;
        uint8_t data[8] = {0};
        uint16_t flags;
        uint8_t enabled, reset;
        uint32_t mask = lock_irq();
        snapshot = key; enabled = board_motion_enabled(); reset = calibrate.flag;
        unlock_irq(mask);
        flags = (BOARD_PROTOCOL_VERSION << 12) | (enabled << 11) | ((reset != 0) << 10) |
                ((snapshot.key_f != 0) << 9) | ((snapshot.key_shift != 0) << 8) |
                ((snapshot.key_d != 0) << 7) | ((snapshot.key_a != 0) << 6) |
                ((snapshot.key_s != 0) << 5) | ((snapshot.key_w != 0) << 4) |
                ((snapshot.btn_l != 0) << 3) | ((snapshot.pause != 0) << 2) | (snapshot.sw & 3u);
        put16(data, (int16_t)snapshot.ch0); put16(data + 2, (int16_t)snapshot.ch1);
        put16(data + 4, (int16_t)flags); data[6] = input_seq + 1u; data[7] = checksum(data);
        if (transmit(&hcan1, BOARD_CHASSIS_INPUT_ID, data))
        {
            uint32_t gap = (uint32_t)(now - input_tx_tick);
            if (input_started && gap > board_diag.chassis_tx_max_gap_ms)
                board_diag.chassis_tx_max_gap_ms = gap;
            board_diag.chassis_tx_last_tick_ms = now;
            ++board_diag.chassis_tx_queued_count;
            input_seq++; input_tx_tick = now; input_started = 1;
        }
    }
    if (!yaw_started || (uint32_t)(now - yaw_tx_tick) >= BOARD_YAW_PERIOD_MS)
    {
        uint8_t data[8] = {0};
        put16(data, yaw_command); data[2] = board_yaw_enabled(); data[3] = yaw_seq + 1u;
        data[4] = BOARD_PROTOCOL_VERSION; data[5] = 0x55; data[6] = 0xAA;
        data[7] = checksum(data);
        if (transmit(&hcan1, BOARD_YAW_COMMAND_ID, data))
        { yaw_seq++; yaw_tx_tick = now; yaw_started = 1; }
    }
#else
    static uint8_t yaw_started, status_started, status_seq;
    static uint32_t yaw_tx_tick, status_tx_tick;
#if BOARD_ROLE == BOARD_LOWER
    lower_diag.motion_enabled = board_motion_enabled();
    lower_diag.yaw_enabled = board_yaw_enabled();
    lower_diag.can2_state = hcan2.State; lower_diag.can2_error = hcan2.ErrorCode;
    lower_diag.can2_mcr = hcan2.Instance->MCR; lower_diag.can2_msr = hcan2.Instance->MSR;
    lower_diag.can2_tsr = hcan2.Instance->TSR; lower_diag.can2_esr = hcan2.Instance->ESR;
#endif
    if (!yaw_started || (uint32_t)(now - yaw_tx_tick) >= BOARD_YAW_PERIOD_MS)
    {
        yaw_started = 1; yaw_tx_tick = now;
        CAN_cmd_yaw(yaw_command);
        /* No chassis current here: the original local CHASSIS_TASK is its sole writer. */
    }
    if (!status_started || (uint32_t)(now - status_tx_tick) >= BOARD_STATUS_PERIOD_MS)
    {
        uint8_t data[8], valid, i;
        uint32_t mask = lock_irq();
        valid = feedback_seen[4] && fresh(feedback_tick[4], BOARD_MOTOR_TIMEOUT_MS);
        memcpy(data, feedback_data[4], 8);
        unlock_irq(mask);
        status_started = 1; status_tx_tick = now;
        if (valid) transmit(&hcan1, BOARD_YAW_FEEDBACK_ID, data);
        memset(data, 0, sizeof(data));
        data[0] = BOARD_PROTOCOL_VERSION; data[1] = ++status_seq;
        for (i = 0; i < 5; ++i)
            if (feedback_seen[i] && fresh(feedback_tick[i], BOARD_MOTOR_TIMEOUT_MS))
                data[2] |= 1u << i;
        data[3] = 0x4C; data[4] = 0x4F; data[5] = 0x57;
        data[7] = checksum(data); board_remote_motor_mask = data[2];
        transmit(&hcan1, BOARD_STATUS_ID, data);
    }
#endif
}
void BOARD_LINK_TASK(void const *argument)
{
    uint32_t wake = osKernelSysTick();
    for (;;)
    {
        board_link_poll(); osDelayUntil(&wake, 1);
    }
}
