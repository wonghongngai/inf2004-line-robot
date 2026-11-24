/*
 *  app_main.c
 *  START HERE.  usermain() below is your program's entry point.
 *
 *  How micro T-Kernel starts your code
 *  -----------------------------------
 *    reset -> kernel initialises itself -> creates the "initial task"
 *          -> initial task calls usermain()
 *
 *  usermain() runs inside that initial task at priority 1 (the highest).
 *  On a single core nothing you create runs until usermain() blocks, but
 *  on the dual-core build the OTHER core may start a new task at once.
 *  So usermain() works in two phases:
 *    1. every subsystem creates its kernel objects  (subsystemN_init)
 *    2. every subsystem starts its tasks             (subsystemN_start)
 *  and then puts the initial task to sleep forever.
 *  If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *  The robot: one subsystem per folder
 *  ------------------------------------
 *    subsystem1/  WiFi/MQTT telemetry and PC commands
 *    subsystem2/  motors, encoders, PID and the Vehicle Controller
 *    subsystem3/  IR line following and barcode
 *    subsystem4/  IMU: terrain, humps, tilt
 *    subsystem5/  ultrasonic obstacle detection and avoidance
 *
 *  The tasks, priorities and cores are in TEAM.md section 4; the
 *  interfaces between subsystems (IF-05 onwards) are in TEAM.md
 *  section 5.  The subsystem that RECEIVES or OWNS the data creates the
 *  object and publishes its ID or access function in its header; the
 *  other side just uses it.
 *
 *  Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"
#if APP_ALL_TEST
#include "all_test/all_buddies_test.h"
#endif
#if APP_LINE_FOLLOW_TEST
#include "all_test/line_follow_test.h"
#endif
#if APP_BARCODE_BENCH
#include "subsystem3/line_service.h"
#include "subsystem3/line_demo.h"
#include "subsystem2/drv_motor.h"
#endif

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char * name)
{
    T_CTSK ctsk = {0};
    ID     tid;
    ER     er;

    ctsk.task    = entry;           /* function the task runs */
    ctsk.itskpri = pri;             /* initial priority (1 = highest) */
    ctsk.stksz   = TASK_STACK_SIZE; /* kernel allocates the stack */
    ctsk.tskatr  = TA_HLNG          /* written in a high-level language (C) */
                  | TA_RNG3;        /* runs at user protection level */
#if TK_SUPPORT_SMP
    if (prc != 0)
    {
        ctsk.tskatr |= TA_ASSPRC; /* honour the affinity below */
        ctsk.assprc = prc;        /* which core(s) it may run on */
    }
#else
    (void)prc; /* single-core build: no affinity */
#endif

    /* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, name);
        return tid;
    }

    /* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
    er = tk_sta_tsk(tid, 0);
    CHECK(er, name);
    if (er < E_OK)
    {
        (void)tk_del_tsk(tid);
        return er;
    }
    /* Maps the task IDs in kernel diagnostics (e.g. S4's FATAL line run=/sched=) to names. */
    LOG_INFO(TAG_APP, "task %s id=%d pri=%d prc=%u", name, (INT)tid, (INT)pri,
             (UINT)prc);
    return tid;
}

EXPORT INT usermain(void)
{
    T_RVER ver;

    /* tk_ref_ver: ask the kernel who it is */
    tk_ref_ver(&ver);
    tm_printf((UB *)"\n"); /* blank line separates boots in the log */
    LOG_INFO(TAG_APP,
             "micro T-Kernel on Raspberry Pi Pico W"
             " (spec 0x%04x, product 0x%04x)",
             ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    LOG_INFO(TAG_APP, "dual-core (SMP) build, usermain on core %d",
             tk_get_prc() - 1); /* tk_get_prc() counts from 1 */
#else
    LOG_INFO(TAG_APP, "single-core build");
#endif

#if APP_ALL_TEST
    ER all_er = all_buddies_test_start();
    CHECK(all_er, "all_buddies_test_start");
    tk_slp_tsk(TMO_FEVR);
    return 0;
#endif

#if APP_LINE_FOLLOW_TEST
    /* Dedicated S2/S3 test owns motors and the IR event queue. */
    ER follow_er = line_follow_test_start();
    CHECK(follow_er, "line_follow_test_start");
    tk_slp_tsk(TMO_FEVR);
    return 0;
#endif

#if APP_BARCODE_BENCH
    /* No motion task or robot adapter: the demo alone owns S3 events. */
    ER bench_er = drv_motor_init();
    if (bench_er == E_OK)
    {
        bench_er = line_service_init();
    }
    if (bench_er == E_OK)
    {
        bench_er = line_service_start();
    }
    if (bench_er == E_OK)
    {
        bench_er = line_demo_start();
    }
    CHECK(bench_er, "barcode_bench_start");
    LOG_INFO(TAG_APP,
             "BARCODE BENCH: motors OFF; scanner GP28, centre GP27, right GP0");
    tk_slp_tsk(TMO_FEVR);
    return 0;
#endif

    /* Phase 1: every subsystem creates its kernel objects */
    ER er = subsystem1_init();
    CHECK(er, "subsystem1_init");
    if (er < E_OK)
    {
        goto startup_failed;
    }
    er = subsystem2_init();
    CHECK(er, "subsystem2_init");
    if (er < E_OK)
    {
        goto startup_failed;
    }
#if APP_ROBOT_INTEGRATED
    er = subsystem3_init();
    CHECK(er, "subsystem3_init");
    if (er < E_OK)
    {
        goto startup_failed;
    }
    er = subsystem4_init();
    CHECK(er, "subsystem4_init");
    if (er < E_OK)
    {
        goto startup_failed;
    }
    er = subsystem5_init();
    CHECK(er, "subsystem5_init");
    if (er < E_OK)
    {
        goto startup_failed;
    }
#endif

    /* Phase 2: every subsystem starts its tasks */
    er = subsystem1_start();
    CHECK(er, "subsystem1_start");
    if (er < E_OK)
    {
        goto startup_failed;
    }
#if APP_ROBOT_INTEGRATED
    er = subsystem3_start();
    CHECK(er, "subsystem3_start");
    if (er < E_OK)
    {
        goto startup_failed;
    }
    er = subsystem4_start();
    CHECK(er, "subsystem4_start");
    if (er < E_OK)
    {
        goto startup_failed;
    }
    er = subsystem5_start();
    CHECK(er, "subsystem5_start");
    if (er < E_OK)
    {
        goto startup_failed;
    }
#endif
    er = subsystem2_start();
    CHECK(er, "subsystem2_start");
    if (er < E_OK)
    {
        goto startup_failed;
    }

#if APP_ROBOT_INTEGRATED
#if S2_MOTOR_HW_TEST
    LOG_INFO(TAG_APP, "INTEGRATED MOTOR TEST: S2 motor pulses + S3 IR + S4 "
                      "IMU; raise wheels");
#else
    LOG_INFO(TAG_APP,
             "INTEGRATED: S2 motion + S3 line/barcode + S4 IMU started");
#endif
#elif S2_MOTOR_HW_TEST
    LOG_INFO(TAG_APP, "MOTOR TEST: S2 only; S3/S4 are not started");
#else
    LOG_INFO(TAG_APP, "STANDALONE MOTION: S2 only; S3/S4 are not started");
#endif

#if APP_TEST
    /* Test build (make TEST=1): run every subsystem's tests in turn,
       here in the initial task, while the system is running.  Tests may
       call tk_dly_tsk() to let the other tasks work for a while. */
    tm_printf(
        (UB *)"[TEST] Connect serial monitor now; tests start in 10 seconds\n");
    tk_dly_tsk(10000);
    tm_printf((UB *)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
#if APP_ROBOT_INTEGRATED
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
#endif
    tm_printf((UB *)"[TEST] === test run end ===\n");
#endif

startup_failed:
    /* Never start motion after a required service failed to start. */
    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
