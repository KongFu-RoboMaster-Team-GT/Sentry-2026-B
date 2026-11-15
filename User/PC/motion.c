#include "motion.h"
#include <string.h>

usb_t usb;
motion_t motion;
volatile pc_diag_t pc_diag;
static volatile uint8_t stop_latched, reset_pending, command_seen, aim_pending;
static volatile uint32_t command_tick;
static uint8_t rx_ring[PC_RX_CAPACITY];
static uint32_t rx_time[PC_RX_CAPACITY];
static volatile uint16_t rx_head, rx_tail;
static uint8_t frame[14], frame_used;
static uint32_t partial_tick, parser_tick;
static uint8_t replies[PC_REPLY_CAPACITY][PC_REPLY_MAX];
static uint8_t reply_len[PC_REPLY_CAPACITY], reply_head, reply_tail, reply_count;
static pc_referee_status_t referee_status = {0, 0, 0, 0, 1};
static float startup_yaw, relative_yaw;
static uint8_t attitude_seen, effective_fire;

static uint32_t pc_lock(void)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq(); return mask;
}
static void pc_unlock(uint32_t mask) { __set_PRIMASK(mask); }
static uint8_t finite_float(float value)
{
    uint32_t bits; memcpy(&bits, &value, 4);
    return (bits & 0x7F800000u) != 0x7F800000u;
}
static void put32le(uint8_t *p, uint32_t value)
{
    uint8_t i;
    for (i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (8u * i));
}
static uint32_t get32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void put_float(uint8_t *p, float value)
{
    uint32_t bits; memcpy(&bits, &value, 4); put32le(p, bits);
}
static float get_float(const uint8_t *p)
{
    uint32_t bits = get32le(p); float value;
    memcpy(&value, &bits, 4); return value;
}
static void clear_command_locked(void)
{
    command_seen = aim_pending = 0;
    usb.deltaYaw = usb.deltaPitch = 0;
    usb.auto_fire_flag = false; motion.auto_fire_detect = 0;
}
void pc_clear_command(void)
{
    uint32_t mask = pc_lock(); clear_command_locked(); pc_unlock(mask);
}
uint8_t pc_stop_active(void) { return stop_latched; }
void pc_poll(void)
{
    uint32_t mask = pc_lock();
    if (command_seen && (uint32_t)(HAL_GetTick() - command_tick) > PC_COMMAND_TIMEOUT_MS) {
        clear_command_locked(); ++pc_diag.command_timeouts;
    }
    pc_unlock(mask);
}
uint8_t pc_fire_requested(void)
{
    uint8_t result;
    uint32_t mask = pc_lock();
    result = !stop_latched && command_seen && usb.auto_fire_flag &&
             (uint32_t)(HAL_GetTick() - command_tick) <= PC_COMMAND_TIMEOUT_MS;
    pc_unlock(mask); return result;
}
uint8_t pc_take_aim(float *yaw_deg, float *pitch_deg)
{
    uint8_t result;
    uint32_t mask = pc_lock();
    result = !stop_latched && command_seen && aim_pending &&
             (uint32_t)(HAL_GetTick() - command_tick) <= PC_COMMAND_TIMEOUT_MS;
    *yaw_deg = result ? usb.deltaYaw : 0;
    *pitch_deg = result ? usb.deltaPitch : 0;
    aim_pending = 0; usb.deltaYaw = usb.deltaPitch = 0;
    pc_unlock(mask); return result;
}
void pc_set_referee_status(const pc_referee_status_t *status)
{
    uint32_t mask = pc_lock(); referee_status = *status; pc_unlock(mask);
}
void pc_note_attitude(float measured_yaw_deg)
{
    uint32_t mask;
    if (!finite_float(measured_yaw_deg) || measured_yaw_deg < -10000 || measured_yaw_deg > 10000) return;
    mask = pc_lock();
    if (!attitude_seen) { startup_yaw = measured_yaw_deg; attitude_seen = 1; }
    relative_yaw = measured_yaw_deg - startup_yaw;
    while (relative_yaw > 180) relative_yaw -= 360;
    while (relative_yaw < -180) relative_yaw += 360;
    pc_unlock(mask);
}
void pc_note_fire(uint8_t enabled) { effective_fire = enabled != 0; }
static void reply(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t *p, i, sum = cmd + len;
    if (reply_count == PC_REPLY_CAPACITY) { ++pc_diag.tx_dropped; return; }
    p = replies[reply_head]; p[0] = 0xAA; p[1] = 0x55; p[2] = cmd; p[3] = len;
    for (i = 0; i < len; ++i) { p[4 + i] = payload[i]; sum += payload[i]; }
    p[4 + len] = sum; reply_len[reply_head] = len + 5u;
    reply_head = (reply_head + 1u) % PC_REPLY_CAPACITY;
    ++reply_count; ++pc_diag.tx_queued;
}
uint8_t pc_tx_peek(uint8_t **data, uint16_t *len)
{
    if (!reply_count) return 0;
    *data = replies[reply_tail]; *len = reply_len[reply_tail]; return 1;
}
void pc_tx_pop(void)
{
    if (!reply_count) return;
    reply_tail = (reply_tail + 1u) % PC_REPLY_CAPACITY;
    --reply_count; ++pc_diag.tx_sent;
}
static uint8_t request_length(uint8_t cmd)
{
    switch (cmd) {
        case 1: return 9;
        case 2: case 3: return 0;
        case 4: return 1;
        default: return 0xFF;
    }
}
static uint8_t dispatch(void)
{
    uint8_t data[14]; uint32_t mask;
    switch (frame[2]) {
        case 1: {
            float yaw_deg = get_float(frame + 4), pitch_deg = get_float(frame + 8);
            if (!finite_float(yaw_deg) || !finite_float(pitch_deg) ||
                yaw_deg < -180 || yaw_deg > 180 || pitch_deg < -180 || pitch_deg > 180 || frame[12] > 1) return 0;
            mask = pc_lock();
            if (!stop_latched) {
                usb.deltaYaw = yaw_deg; usb.deltaPitch = pitch_deg;
                usb.auto_fire_flag = frame[12] != 0; motion.auto_fire_detect = frame[12];
                command_tick = parser_tick; command_seen = aim_pending = 1;
            }
            pc_unlock(mask);
            ++pc_diag.aim_commands; reply(1, data, 0); break;
        }
        case 2:
            mask = pc_lock();
            put32le(data, (uint32_t)referee_status.health);
            put32le(data + 4, (uint32_t)referee_status.fire_number);
            put_float(data + 8, referee_status.heat);
            data[12] = referee_status.zone_is_ours != 0; data[13] = referee_status.stop != 0;
            pc_unlock(mask); reply(2, data, 14); break;
        case 3:
            mask = pc_lock(); put_float(data, relative_yaw);
            data[4] = effective_fire && !stop_latched;
            pc_unlock(mask); reply(3, data, 5); break;
        case 4:
            if (frame[4] > 1) return 0;
            mask = pc_lock(); stop_latched = frame[4]; clear_command_locked();
            effective_fire = 0; pc_unlock(mask); ++pc_diag.stop_commands; break;
        default: return 0;
    }
    return 1;
}
static void discard_byte(void)
{
    uint8_t i; --frame_used;
    for (i = 0; i < frame_used; ++i) frame[i] = frame[i + 1u];
}
static uint8_t parse_bytes(const uint8_t *data, uint16_t len, uint32_t received_tick)
{
    uint16_t n; uint8_t accepted = 0;
    if (!data || !len || reset_pending) return 0;
    parser_tick = received_tick;
    if (frame_used && (uint32_t)(received_tick - partial_tick) > PC_PARTIAL_TIMEOUT_MS)
    { frame_used = 0; ++pc_diag.partial_timeouts; }
    partial_tick = received_tick;
    for (n = 0; n < len; ++n) {
        if (reset_pending) break;
        frame[frame_used++] = data[n];
        while (frame_used) {
            uint8_t i, sum = 0, total;
            if (frame[0] != 0xAA || (frame_used >= 2 && frame[1] != 0x55))
            { discard_byte(); continue; }
            if (frame_used < 4) break;
            if (request_length(frame[2]) == 0xFF || request_length(frame[2]) != frame[3])
            { ++pc_diag.format_errors; discard_byte(); continue; }
            total = frame[3] + 5u;
            if (frame_used < total) break;
            for (i = 2; i < total - 1u; ++i) sum += frame[i];
            if (sum != frame[total - 1u])
            { ++pc_diag.checksum_errors; discard_byte(); continue; }
            {
                uint32_t mask = pc_lock();
                if (!reset_pending) {
                    if (dispatch()) { accepted = 1; ++pc_diag.valid_frames; }
                    else ++pc_diag.format_errors;
                }
                pc_unlock(mask);
            }
            for (i = total; i < frame_used; ++i) frame[i - total] = frame[i];
            frame_used -= total;
        }
    }
    return accepted;
}
uint8_t usbreceive(const uint8_t *data, uint16_t len)
{
    return parse_bytes(data, len, HAL_GetTick());
}
void pc_link_reset_isr(void)
{
    uint32_t mask = pc_lock(); reset_pending = 1; rx_tail = rx_head;
    clear_command_locked(); effective_fire = 0; pc_unlock(mask);
}
void pc_receive_isr(const uint8_t *data, uint16_t len)
{
    uint16_t i, free_bytes;
    uint32_t mask = pc_lock();
    if (!data || reset_pending) { pc_unlock(mask); return; }
    free_bytes = (rx_tail + PC_RX_CAPACITY - rx_head - 1u) % PC_RX_CAPACITY;
    if (len > free_bytes) {
        ++pc_diag.rx_overflows; reset_pending = 1; rx_tail = rx_head;
        clear_command_locked(); effective_fire = 0; pc_unlock(mask); return;
    }
    for (i = 0; i < len; ++i) {
        rx_ring[rx_head] = data[i]; rx_time[rx_head] = HAL_GetTick();
        rx_head = (rx_head + 1u) % PC_RX_CAPACITY;
    }
    pc_diag.rx_bytes += len; pc_unlock(mask);
}
void pc_process_rx(void)
{
    uint16_t budget = PC_RX_CAPACITY;
    while (budget) {
        uint8_t chunk[64]; uint16_t count = 0; uint32_t received_tick;
        uint32_t mask = pc_lock();
        if (reset_pending) {
            frame_used = reply_head = reply_tail = reply_count = 0;
            rx_tail = rx_head; reset_pending = 0; ++pc_diag.link_resets;
        }
        received_tick = rx_time[rx_tail];
        while (rx_tail != rx_head && count < sizeof(chunk) && count < budget &&
               rx_time[rx_tail] == received_tick) {
            chunk[count++] = rx_ring[rx_tail]; rx_tail = (rx_tail + 1u) % PC_RX_CAPACITY;
        }
        pc_unlock(mask);
        if (!count) break;
        if ((uint32_t)(HAL_GetTick() - received_tick) > PC_COMMAND_TIMEOUT_MS) {
            frame_used = 0; pc_clear_command(); ++pc_diag.rx_expired;
        } else parse_bytes(chunk, count, received_tick);
        budget -= count;
    }
    pc_poll();
}
