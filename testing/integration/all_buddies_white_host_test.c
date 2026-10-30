#define ALL_TEST_WHITE_FOLLOW 1
#define ALL_TEST_STOP_ON_COLLISION 1
#define main legacy_test_main
#include "all_buddies_host_test.c"
#undef main

int main(void)
{
    for (unsigned mask = 0; mask < 8; ++mask) {
        ALL_TEST t = setup();
        fake_line.data[0] = mask;
        step(&t, 5000); step(&t, 5030);
        switch (mask & 3u) {
        case 0: EXPECT(t.phase == RUNNING && motor_left == 12.5f && motor_right == 14.2f); break;
        case 1: EXPECT(t.phase == RUNNING && motor_left == 20.0f && motor_right == 0); break;
        case 2: EXPECT(t.phase == RUNNING && motor_left == 0 && motor_right == 14.2f); break;
        case 3: EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0); break;
        }
    }
    ALL_TEST t = setup(); fake_line.data[0] = 0;
    step(&t, 5000); step(&t, 5030);
    fake_line.data[1] = LINE_BARCODE_SEARCH;
    fake_event.data[1] = LINE_BARCODE_EVT_LOST << 3; has_event = true;
    step(&t, 15040);
    EXPECT(t.phase == RUNNING && motor_left == 12.5f && motor_right == 14.2f);
    fake_line.data[0] = 1; step(&t, 15050);
    EXPECT(motor_left == 20.0f && motor_right == 0);
    fake_line.data[0] = 2; step(&t, 15060);
    EXPECT(motor_left == 0 && motor_right == 14.2f);
    fake_line.data[0] = 4; fake_line.data[1] = LINE_BARCODE_JUNCTION;
    fake_event.data[1] = LINE_BARCODE_EVT_JUNCTION << 3; has_event = true;
    step(&t, 15070);
    EXPECT(t.phase == RUNNING && motor_left == 0 && motor_right == 14.2f);
    step(&t, 15170);
    EXPECT(motor_left == 12.5f && motor_right == 14.2f);
    command = S1_COMMAND_PAUSE; step(&t, 15180);
    EXPECT(t.phase == PAUSED && motor_left == 0 && motor_right == 0);
    t = setup(); fake_line.data[0] = 0; step(&t, 5000); step(&t, 5030);
    fake_imu.event = IMU_EVENT_COLLISION; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    t = setup(); fake_line.data[0] = 0; step(&t, 5000); step(&t, 5030);
    line_error = E_TMOUT; step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    for (unsigned direction = 1; direction <= 2; ++direction) {
        t = setup(); fake_line.data[0] = 0; step(&t, 5000); step(&t, 5030);
        fake_line.data[0] = direction; step(&t, 5040);
        fake_line.data[0] = 0; step(&t, 5050); step(&t, 5149);
        EXPECT(t.steering_action == direction);
        step(&t, 5150);
        EXPECT(t.steering_action == 0 && motor_left == 12.5f && motor_right == 14.2f);
        fake_line.data[0] = direction; step(&t, 5160);
        fake_line.data[0] = 0; step(&t, 5170);
        fake_line.data[0] = direction; step(&t, 5220); /* Flicker resets confirmation. */
        fake_line.data[0] = 0; step(&t, 5230); step(&t, 5329);
        EXPECT(t.steering_action == direction);
        step(&t, 5330); EXPECT(t.steering_action == 0);
    }
    fake_line.data[0] = 1; step(&t, 5340);
    fake_line.data[0] = 2; step(&t, 5350);
    EXPECT(t.steering_action == 2 && motor_left == 0 && motor_right == 14.2f);
    command = S1_COMMAND_PAUSE; step(&t, 5360);
    EXPECT(t.phase == PAUSED && t.steering_action == 0 && motor_left == 0 && motor_right == 0);
    printf("White centre/right tracking: %u checks passed\n", checks);
    return 0;
}
