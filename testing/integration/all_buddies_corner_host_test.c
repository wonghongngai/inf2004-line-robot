#define ALL_TEST_CORNER_SEARCH 1
#define ALL_TEST_STOP_ON_COLLISION 1
#define main legacy_test_main
#include "all_buddies_host_test.c"
#undef main

int main(void)
{
    ALL_TEST t = running();
    EXPECT(motor_left == 12.5f && motor_right == 14.2f);
    fake_line.data[0] = 4; step(&t, 5040);
    EXPECT(t.corner_active && t.corner_direction == -1);
    EXPECT(motor_left == 0 && motor_right == 14.2f);
    fake_line.data[0] = 0; fake_line.data[1] = LINE_BARCODE_SEARCH;
    fake_event.data[1] = LINE_BARCODE_EVT_LOST << 3; has_event = true;
    step(&t, 5140);
    EXPECT(t.phase == RUNNING && motor_left == 0 && motor_right > 0);
    fake_line.data[0] = 2; fake_line.data[1] = LINE_BARCODE_FOLLOW;
    step(&t, 5150); EXPECT(t.corner_active && motor_left > 0 && motor_right > 0);
    fake_line.data[0] = 0; step(&t, 5180); EXPECT(!t.centre_pending);
    fake_line.data[0] = 2; step(&t, 5190); step(&t, 5240);
    EXPECT(t.corner_active); step(&t, 5250); EXPECT(!t.corner_active);
    step(&t, 5260); EXPECT(motor_left == 12.5f && motor_right == 14.2f);

    t = running(); fake_line.data[0] = 1; step(&t, 5040);
    EXPECT(t.corner_active && motor_left == 12.5f && motor_right == 0);
    fake_line.data[0] = 0; step(&t, 10039); EXPECT(t.phase == RUNNING);
    step(&t, 10040); EXPECT(t.phase == RUNNING && t.corner_active);
    step(&t, 65040); EXPECT(t.phase == RUNNING && motor_left > 0 && motor_right == 0);
    fake_line.data[0] = 2; step(&t, 65100); step(&t, 65160);
    EXPECT(t.phase == RUNNING && !t.corner_active);

    t = running(); fake_line.data[0] = 0; step(&t, 5040);
    EXPECT(t.corner_active && t.corner_direction == -1 && motor_left == 0 && motor_right > 0);
    t = running(); fake_line.data[0] = 6; step(&t, 5040);
    fake_line.data[0] = 0; step(&t, 5050);
    EXPECT(t.corner_active && motor_left == 0 && motor_right > 0);
    t = running(); fake_line.data[0] = 6; step(&t, 5040);
    fake_line.data[0] = 2; step(&t, 5050); fake_line.data[0] = 0; step(&t, 5060);
    EXPECT(t.corner_active && t.corner_direction == -1);

    for (unsigned mask = 5; mask <= 7; mask += 2) {
        t = running(); fake_line.data[0] = 4; step(&t, 5040);
        fake_line.data[0] = mask; step(&t, 5050);
        EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    }
    t = running(); fake_line.data[0] = 4; step(&t, 5040);
    fake_line.data[0] = 1; step(&t, 5050);
    EXPECT(t.phase == RUNNING && t.corner_direction == 1 && motor_left > 0 && motor_right == 0);
    EXPECT(t.corner_started == 5040);
    fake_line.data[0] = 0; step(&t, 5060);
    EXPECT(t.corner_direction == 1 && motor_left > 0 && motor_right == 0);
    fake_line.data[0] = 3; step(&t, 5070);
    EXPECT(t.corner_direction == 1 && motor_left > 0 && motor_right == 0);
    step(&t, 10040); EXPECT(t.phase == RUNNING && t.corner_active);
    t = running(); fake_line.data[0] = 4; step(&t, 5040);
    command = S1_COMMAND_PAUSE; step(&t, 5050);
    EXPECT(t.phase == PAUSED && !t.corner_active && motor_left == 0 && motor_right == 0);
    t = running(); fake_line.data[0] = 4; step(&t, 5040);
    fake_imu.motion_state = IMU_MOTION_TILT; step(&t, 5050);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    t = running(); fake_line.data[0] = 1; step(&t, 5040);
    fake_imu.event = IMU_EVENT_COLLISION; step(&t, 5050);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    t = running(); fake_line.data[0] = 1; step(&t, 5040);
    line_error = E_TMOUT; step(&t, 5050);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    t = running(); fake_line.data[0] = 1; step(&t, 5040);
    receive_error = E_PAR; step(&t, 5050); EXPECT(t.phase == STOPPED);
    t = running(); fake_line.data[0] = 6; step(&t, 5040);
    fake_line.data[0] = 0; step(&t, 5341);
    EXPECT(t.corner_active && t.corner_direction == -1); /* All-white always searches left. */
    t = running(); t.pwm_ramp_per_second = ALL_TEST_PWM_RAMP_PER_SECOND;
    fake_line.data[0] = 4; step(&t, 5040);
    EXPECT(motor_left == 0 && motor_right <= 14.2f);
    fake_line.data[0] = 2; step(&t, 5050);
    EXPECT(motor_left > 0 && motor_right > 0);
    step(&t, 5110); step(&t, 5120);
    EXPECT(!t.corner_active && motor_left > 0 && motor_left <= 12.5f &&
           motor_right > 0 && motor_right <= 14.2f); /* Confirmation continues without resetting PWM. */
    t = running(); fake_line.data[0] = 4; step(&t, 5040);
    fake_line.data[0] = 0; all_test_step(&t, 5141); /* IMU becomes stale. */
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    t = running(); fake_line.data[0] = 1; step(&t, 5040);
    fake_line.data[0] = 0; step(&t, 5140);
    EXPECT(t.corner_direction == 1 && motor_left > 0 && motor_right == 0);
    fake_line.data[0] = 1; step(&t, 5240); /* Refresh side memory. */
    fake_line.data[0] = 0; step(&t, 5540);
    EXPECT(t.corner_direction == 1);
    step(&t, 5541); EXPECT(t.corner_direction == 1);
    fake_line.data[0] = 1; step(&t, 5550);
    EXPECT(t.corner_direction == 1);
    fake_line.data[0] = 4; step(&t, 5560);
    EXPECT(t.corner_direction == -1); /* Actual opposite black wins immediately. */
    t = running(); fake_line.data[0] = 1; step(&t, 5040);
    fake_line.data[0] = 0; step(&t, 15040); step(&t, 65040);
    EXPECT(t.corner_active && t.corner_direction == 1 &&
           motor_left == 12.5f && motor_right == 0);
    fake_line.data[0] = 2; step(&t, 65050); step(&t, 65110);
    EXPECT(!t.corner_active && t.phase == RUNNING);
    fake_line.data[0] = 0; step(&t, 65120);
    EXPECT(t.corner_active && t.corner_direction == -1); /* New search, expired memory. */
    printf("Corner search: %u checks passed\n", checks);
    return 0;
}
