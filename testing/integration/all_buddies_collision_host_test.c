/* Validate the combined driving profile's collision stop configuration. */
#define ALL_TEST_STOP_ON_COLLISION 1
#define main legacy_test_main
#include "all_buddies_host_test.c"
#undef main

int main(void)
{
    ALL_TEST t = running();
    EXPECT(motor_left == 12.5f && motor_right == 14.2f);
    fake_imu.event = IMU_EVENT_COLLISION;
    step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    fake_imu.event = IMU_EVENT_NONE;
    step(&t, 5050);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    t = running();
    fake_imu.motion_state = IMU_MOTION_COLLISION;
    step(&t, 5040);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    printf("Collision stop profile: %u checks passed\n", checks);
    return 0;
}
