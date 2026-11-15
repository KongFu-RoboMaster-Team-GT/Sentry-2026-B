/* Included by protocol_arm.c: exercise actual parser, queue and control gates. */
static const uint8_t aim_example[] = {0xAA,0x55,1,9,0,0,0x48,0x41,0,0,0x70,0xC0,1,0xC4};
static const uint8_t status_query[] = {0xAA,0x55,2,0,2};
static const uint8_t attitude_query[] = {0xAA,0x55,3,0,3};
static const uint8_t stop_on[] = {0xAA,0x55,4,1,1,6};
static const uint8_t stop_off[] = {0xAA,0x55,4,1,0,5};
static void serial_reset(void)
{
    pc_link_reset_isr(); pc_process_rx();
    CHECK(usbreceive(stop_off, sizeof(stop_off)));
}
static void serial_reply(uint8_t cmd, uint16_t size)
{
    uint8_t *p, i, sum = 0; uint16_t len;
    CHECK(pc_tx_peek(&p, &len)); CHECK(len == size);
    CHECK(p[0] == 0xAA && p[1] == 0x55 && p[2] == cmd && p[3] == size - 5);
    for (i = 2; i < len - 1; ++i) sum += p[i];
    CHECK(sum == p[len - 1]); pc_tx_pop();
}
static void test_serial_protocol(void)
{
    uint32_t i, j, before; uint8_t *p, bad[64], packet[14]; uint16_t len;
    float yaw_deg, pitch_deg;
    pc_referee_status_t state = {257, 513, 37.5f, 0, 1};
    tick = 100;
    serial_reset();
    CHECK(!usbreceive(NULL, 100)); CHECK(!usbreceive(aim_example, 0));
    /* Golden frame and every possible USB segmentation. */
    for (i = 1; i < sizeof(aim_example); ++i) {
        serial_reset();
        CHECK(!usbreceive(aim_example, (uint16_t)i));
        CHECK(usbreceive(aim_example + i, sizeof(aim_example) - i));
        CHECK(pc_take_aim(&yaw_deg, &pitch_deg));
        CHECK(yaw_deg == 12.5f && pitch_deg == -3.75f && pc_fire_requested());
        CHECK(!pc_take_aim(&yaw_deg, &pitch_deg));
        serial_reply(1, 5); CHECK(!pc_tx_peek(&p, &len));
    }
    serial_reset();
    for (i = 0; i < sizeof(aim_example); ++i) usbreceive(aim_example + i, 1);
    CHECK(pc_take_aim(&yaw_deg, &pitch_deg) && yaw_deg == 12.5f); serial_reply(1, 5);
    /* No single corrupted bit may execute a command. Following valid data resyncs. */
    for (i = 0; i < sizeof(aim_example); ++i) for (j = 0; j < 8; ++j) {
        serial_reset(); memcpy(packet, aim_example, sizeof(packet)); packet[i] ^= 1u << j;
        usbreceive(packet, sizeof(packet)); CHECK(!pc_fire_requested());
        CHECK(!pc_tx_peek(&p, &len));
        usbreceive(aim_example, sizeof(aim_example));
        CHECK(pc_fire_requested()); serial_reply(1, 5);
    }
    serial_reset();
    /* Unknown CMD=FF/LEN=FF must not wait for an oversized payload. */
    bad[0]=0xAA; bad[1]=0x55; bad[2]=0xFF; bad[3]=0xFF;
    usbreceive(bad, 4); CHECK(usbreceive(status_query, 5)); serial_reply(2, 19);
    /* Embedded valid frames inside a bad frame, preserving trailing bytes. */
    serial_reset();
    bad[0]=0xAA; bad[1]=0x55; bad[2]=1; bad[3]=9;
    memcpy(bad+4, status_query, 5); memcpy(bad+9, attitude_query, 5);
    usbreceive(bad, 14); serial_reply(2, 19); serial_reply(3, 10);
    /* Referee wire layout is explicit int32/float32/bool, no struct padding. */
    pc_set_referee_status(&state); CHECK(usbreceive(status_query, 5));
    CHECK(pc_tx_peek(&p, &len));
    CHECK(len == 19 && p[4]==1 && p[5]==1 && p[6]==0 && p[7]==0);
    CHECK(p[8]==1 && p[9]==2 && p[10]==0 && p[11]==0);
    CHECK(get_float(p+12)==37.5f && p[16]==0 && p[17]==1); serial_reply(2,19);
    pc_note_attitude(170); pc_note_attitude(-170); pc_note_fire(1);
    CHECK(usbreceive(attitude_query,5)); CHECK(pc_tx_peek(&p,&len));
    CHECK(get_float(p+4)==20 && p[8]==1); serial_reply(3,10);
    /* NaN, infinity, out-of-range deltas and noncanonical bools are rejected. */
    for (i = 0; i < 4; ++i) {
        uint8_t sum = 0;
        serial_reset(); memcpy(packet,aim_example,14);
        if (i==0) put32le(packet+4,0x7FC00000u);
        if (i==1) put32le(packet+8,0x7F800000u);
        if (i==2) put_float(packet+4,181);
        if (i==3) packet[12]=2;
        for (j=2;j<13;++j) sum+=packet[j]; packet[13]=sum;
        CHECK(!usbreceive(packet,14)); CHECK(!pc_fire_requested()); CHECK(!pc_tx_peek(&p,&len));
    }
    serial_reset();
    usbreceive(aim_example,5); tick+=51; CHECK(!usbreceive(aim_example+5,9));
    CHECK(!pc_fire_requested()); CHECK(usbreceive(aim_example,14)); serial_reply(1,5);
    tick+=200; CHECK(pc_fire_requested()); tick+=1; CHECK(!pc_fire_requested());
    CHECK(!pc_take_aim(&yaw_deg,&pitch_deg)); pc_poll(); CHECK(!usb.auto_fire_flag);
    /* HAL tick wrap must preserve the freshness window. Queries don't refresh it. */
    tick=0xFFFFFFF0u; usbreceive(aim_example,14); serial_reply(1,5);
    tick=0x20u; CHECK(pc_fire_requested()); usbreceive(status_query,5); serial_reply(2,19);
    tick=0x100u; CHECK(!pc_fire_requested());
    /* Stop survives USB resets and blocked aims; release clears old commands. */
    serial_reset(); usbreceive(aim_example,14); serial_reply(1,5);
    CHECK(usbreceive(stop_on,6)); CHECK(pc_stop_active() && !pc_fire_requested());
#if BOARD_ROLE == BOARD_UPPER
    board_note_input(); CHECK(!board_motion_enabled() && !board_yaw_enabled());
    CAN_cmd_pitch(1000); CHECK(get16(last_data+2)==0);
    CAN_cmd_shoot(100,200,300); CHECK(get16(last_data)==0 && get16(last_data+4)==0);
#endif
    usbreceive(aim_example,14); CHECK(!pc_take_aim(&yaw_deg,&pitch_deg)); serial_reply(1,5);
    pc_link_reset_isr(); pc_process_rx(); CHECK(pc_stop_active());
    CHECK(!pc_tx_peek(&p,&len)); CHECK(usbreceive(stop_off,6));
    CHECK(!pc_stop_active() && !pc_fire_requested() && !pc_tx_peek(&p,&len));
#if BOARD_ROLE == BOARD_UPPER
    /* Actual gimbal KM handler consumes a delta once, then manual mode discards it. */
    key.mouse_btn_r=1; gimbal.target_yaw=0; gimbal.target_pitch=0;
    usbreceive(aim_example,14); test_gimbal_km_control(&gimbal);
    CHECK(gimbal.target_yaw==12.5f && gimbal.target_pitch==-3.75f);
    test_gimbal_km_control(&gimbal); CHECK(gimbal.target_yaw==12.5f);
    key.mouse_btn_r=0; key.mouse_x=key.mouse_y=0;
    usbreceive(aim_example,14); test_gimbal_km_control(&gimbal);
    key.mouse_btn_r=1; test_gimbal_km_control(&gimbal); CHECK(gimbal.target_yaw==12.5f);
    test_shooter_km_control(&key,&motion,&shooter_control); CHECK(shooter_control.shoot_flag==1);
    tick+=201; test_shooter_km_control(&key,&motion,&shooter_control); CHECK(!shooter_control.shoot_flag);
    key.mouse_btn_r=0;
#endif
    serial_reset();
    /* An ISR copy remains valid after USB overwrites its original buffer. */
    memcpy(packet,aim_example,14); pc_receive_isr(packet,14); memset(packet,0,14);
    pc_process_rx(); CHECK(pc_fire_requested()); serial_reply(1,5);
    serial_reset();
    pc_receive_isr(aim_example,14); tick+=201; pc_process_rx();
    CHECK(!pc_fire_requested() && !pc_tx_peek(&p,&len) && pc_diag.rx_expired>0);
    for (i=0;i<20;++i) { pc_receive_isr(status_query,5); pc_process_rx(); serial_reply(2,19); }
    /* RX ring wraps repeatedly without corrupting concatenated frames. */
    for (i=0;i<100;++i) {
        memcpy(bad,status_query,5); memcpy(bad+5,attitude_query,5);
        pc_receive_isr(bad,10); pc_process_rx(); serial_reply(2,19); serial_reply(3,10);
    }
    before=pc_diag.rx_overflows;
    memset(bad,0,sizeof(bad));
    for(i=0;i<8;++i) pc_receive_isr(bad,64);
    CHECK(pc_diag.rx_overflows==before+1 && !pc_fire_requested());
    pc_process_rx(); pc_receive_isr(status_query,5); pc_process_rx(); serial_reply(2,19);
    /* TX full is observable and cannot prevent CMD04. Peeks emulate BUSY retries. */
    serial_reset();
    for (i=0;i<PC_REPLY_CAPACITY+1;++i) usbreceive(status_query,5);
    CHECK(pc_diag.tx_dropped>0); CHECK(pc_tx_peek(&p,&len));
    CHECK(len==19 && p[2]==2); CHECK(pc_tx_peek(&p,&len));
    CHECK(usbreceive(stop_on,6) && pc_stop_active());
    for (i=0;i<PC_REPLY_CAPACITY;++i) serial_reply(2,19);
    CHECK(!pc_tx_peek(&p,&len));
    serial_reset();
    /* Restore global fixture state for the existing board/control regression. */
    attitude_seen=effective_fire=0; startup_yaw=relative_yaw=0;
    memset((void *)&pc_diag,0,sizeof(pc_diag));
#if BOARD_ROLE == BOARD_UPPER
    input_seen=0; board_armed=0; tx_count=0;
    memset(&key,0,sizeof(key)); memset(&gimbal,0,sizeof(gimbal));
#endif
}
