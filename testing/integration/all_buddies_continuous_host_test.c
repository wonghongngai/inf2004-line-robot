#define ALL_TEST_CONTINUE_ON_SENSOR 1
#define main legacy_test_main
#include "all_buddies_host_test.c"
#undef main

int main(void)
{
    ALL_TEST t = setup();
    fake_imu.calibrated = false;
    step(&t, 5000);
    EXPECT(t.phase == RUNNING && motor_left > 0 && motor_right > 0);
    for (unsigned state = 0; state <= IMU_MOTION_FAULT; ++state) {
        fake_imu.motion_state = state;
        fake_imu.event = state;
        for (unsigned mask = 0; mask < 8; ++mask) {
            fake_line.data[0] = mask;
            fake_line.data[1] = LINE_BARCODE_FAULT;
            fake_event.data[1] = LINE_BARCODE_EVT_LOST << 3;
            has_event = true;
            step(&t, 5010 + state * 100 + mask * 10);
            EXPECT(t.phase == RUNNING);
            if (mask == 0 || mask == 5 || mask == 7)
                EXPECT(motor_left == 0 && motor_right == 0);
            else
                EXPECT(motor_left > 0 && motor_right > 0);
        }
    }
    fake_line.data[0] = 2; step(&t, 6500);
    EXPECT(motor_left > 0 && motor_right > 0); /* Auto-resume on black. */
    t.pwm_ramp_per_second = ALL_TEST_PWM_RAMP_PER_SECOND;
    fake_line.data[0] = 0; step(&t, 6510);
    EXPECT(motor_left == 0 && motor_right == 0); /* Immediate stop, no ramp. */
    fake_line.data[0] = 2; step(&t, 6520);
    EXPECT(motor_left == 1 && motor_right == 1); /* Smooth restart. */
    imu_error = line_error = receive_error = enable_error = E_PAR;
    fake_imu.sensor_ok = false;
    ++fake_imu.read_errors;
    all_test_step(&t, 7000); /* Stale samples and broken APIs. */
    EXPECT(t.phase == RUNNING && motor_left == 0 && motor_right == 0);
    command = S1_COMMAND_PAUSE; step(&t, 7010);
    EXPECT(t.phase == PAUSED && motor_left == 0 && motor_right == 0);
    line_error = E_OK; fake_line.data[0] = 2;
    command = S1_COMMAND_RESUME; step(&t, 7020);
    EXPECT(t.phase == RUNNING && motor_left > 0 && motor_right > 0);
    t.duration_ms = 1; step(&t, 7030);
    EXPECT(t.phase == STOPPED && motor_left == 0 && motor_right == 0);
    printf("Continuous sensor mode: %u checks passed\n", checks);
    return 0;
}
