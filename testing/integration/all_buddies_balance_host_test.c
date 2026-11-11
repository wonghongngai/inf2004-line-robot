/* Exercise production straight-wheel feedback using simulated encoders. */
#define main legacy_test_main
#include "all_buddies_host_test.c"
#undef main
#include "../../app_program/subsystem1/communication_protocol.c"

/* Unused service boundaries required by the older coordinator harness. */
uint32_t line_service_fault_interval_ms(void) { return 0; }
ER line_service_lookup(uint8_t letter, uint8_t *command)
{ (void)letter; (void)command; return E_PAR; }
ER line_service_set_mapping(char l, char r, char s, char u)
{ (void)l; (void)r; (void)s; (void)u; return E_OK; }
INT subsystem1_receive_mapping(S1_MAPPING *m, TMO timeout)
{ (void)m; (void)timeout; return E_TMOUT; }
bool line_service_barcode_run(uint32_t *ms, unsigned *black, uint32_t *dropped)
{ (void)ms; (void)black; (void)dropped; return false; }
void line_service_barcode_raw(unsigned *level, unsigned *rises, unsigned *falls)
{ *level = *rises = *falls = 0; }
void line_service_barcode_level(unsigned *black, uint32_t *age)
{ *black = 0; *age = 0; }
void line_service_barcode_debug(unsigned *black, unsigned *white, uint8_t *letter)
{ *black = *white = 0; *letter = 0; }

int main(void)
{
    ALL_TEST t = {0};
    wheel_odometry_init(&t.odometry, S2_LEFT_TICKS_PER_CM,
                        S2_RIGHT_TICKS_PER_CM, 0, 0);
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, 100);
    EXPECT(motor_left == ALL_BASE_PWM && motor_right == ALL_RIGHT_BASE_PWM);
    /* Forward counts can have either polarity. Left travels farther. */
    t.left_ticks = -46;
    t.right_ticks = 19;
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, 110);
    EXPECT(motor_left < ALL_BASE_PWM && motor_right > ALL_RIGHT_BASE_PWM);
    t.left_ticks = -10000;
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, 120);
    EXPECT(motor_left == ALL_BASE_PWM - ALL_TEST_STRAIGHT_MAX_TRIM);
    EXPECT(motor_right == ALL_RIGHT_BASE_PWM + ALL_TEST_STRAIGHT_MAX_TRIM);
    /* A steering command clears feedback and keeps the pivot unchanged. */
    apply_follow_pwm(&t, ALL_BASE_PWM, 0, 130);
    EXPECT(!t.straight_active && t.straight_error_cm == 0);
    EXPECT(motor_left == ALL_BASE_PWM && motor_right == 0);
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, 140);
    EXPECT(motor_left == ALL_BASE_PWM && motor_right == ALL_RIGHT_BASE_PWM);
    t.right_ticks += 100;
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, 150);
    EXPECT(motor_left > ALL_BASE_PWM && motor_right < ALL_RIGHT_BASE_PWM);
    /* A stopped output or control gap must rebase before resuming. */
    t.left = t.right = 0;
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, 160);
    EXPECT(t.straight_error_cm == 0);
    t.left_ticks -= 100;
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, 1000);
    EXPECT(t.straight_error_cm == 0);
    t.odometry.available = false;
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, 1010);
    EXPECT(!t.straight_active && motor_left == ALL_BASE_PWM);
    printf("Straight encoder balance: %u checks passed\n", checks);
    return 0;
}
