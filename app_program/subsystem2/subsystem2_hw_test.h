/* Opt-in motor bench test. This mode physically moves the wheels at boot. */
#ifndef S2_HW_TEST_H
#define S2_HW_TEST_H

/* Set to 1 only for the raised-wheel bench test; restore to 0 afterwards. */
#ifndef S2_MOTOR_HW_TEST
#define S2_MOTOR_HW_TEST 0
#endif

#define S2_HW_TEST_PWM_PERCENT 20.0f
#define S2_HW_TEST_RUN_MS      750U
#define S2_HW_TEST_PAUSE_MS    1500U
#define S2_HW_TEST_START_MS    5000U

#if (S2_MOTOR_HW_TEST != 0) && (S2_MOTOR_HW_TEST != 1)
#error "S2_MOTOR_HW_TEST must be 0 or 1"
#endif

#if S2_MOTOR_HW_TEST
IMPORT void subsystem2_hw_test_task(INT stacd, void *exinf);
#endif

#endif
