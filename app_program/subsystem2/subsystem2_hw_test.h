/* Opt-in motor bench test. This mode physically moves the wheels at boot. */
#ifndef S2_HW_TEST_H
#define S2_HW_TEST_H

/* Set to 1 only for the raised-wheel bench test; restore to 0 afterwards. */
#ifndef S2_MOTOR_HW_TEST
#define S2_MOTOR_HW_TEST 0
#endif

/* Floor bench for S2-U10 (straight PID) and S2-U11 (STOP latency), #18.
   Build with `sh build.sh motion MOTION_DEMO=1 PID_BENCH=1`; needs 1.2 m
   of clear floor. Replaces the demo sequence. */
#ifndef S2_PID_BENCH
#define S2_PID_BENCH 0
#endif
#define S2_BENCH_STOP_AFTER_MS 1000U
#define S2_PID_BENCH_PWM         25.0f   /* 55 % default is too fast; 15 % stalls on the floor */

/* Floor bench for S2-U07: motor-driven pivots, see robot_control.c. Build with
   `sh build.sh motion MOTION_DEMO=1 PIVOT_BENCH=1`. Not with PID_BENCH. */
#ifndef S2_PIVOT_BENCH
#define S2_PIVOT_BENCH 0
#endif
#define S2_PIVOT_BENCH_PWM 25.0f   /* 15 % stalled the left wheel on the floor (22 Nov) */
#if S2_PID_BENCH && S2_PIVOT_BENCH
#error "S2_PID_BENCH and S2_PIVOT_BENCH are mutually exclusive"
#endif

#if S2_PID_BENCH
IMPORT volatile uint32_t s2_bench_stop_ms;
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
