#include "subsystem5.h"
#include "drv_rp2040.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

static const UW g_test_angles_deg[] = {90U, 60U,  30U,  60U,
                                       90U, 120U, 150U, 120U};
#define TEST_ANGLE_COUNT \
    (sizeof(g_test_angles_deg) / sizeof(g_test_angles_deg[0]))

static const char * status_name(ultrasonic_status_t status)
{
    switch (status)
    {
        case ULTRASONIC_OK: return "ok";
        case ULTRASONIC_BUSY: return "busy";
        case ULTRASONIC_NO_ECHO: return "no-echo";
        case ULTRASONIC_TOO_NEAR: return "too-near";
        case ULTRASONIC_TOO_FAR: return "too-far";
        default: return "?";
    }
}

EXPORT void obstacle_task(INT stacd, void * exinf)
{
    UW                  index;
    UW                  distance_mm;
    ultrasonic_status_t status;

    /* To hide unused warnings */
    (void)stacd;
    (void)exinf;

    ultrasonic_init();
    servo_init();

    /* A task must never return: loop forever */
    for (;;)
    {
        for (index = 0U; index < TEST_ANGLE_COUNT; index++)
        {
            servo_set_angle(g_test_angles_deg[index]);
            tk_dly_tsk(1000); /* let the servo arrive before pinging */

            status = ultrasonic_measure_mm(&distance_mm);
            tm_printf((UB *)"S5: angle %u  d=%u mm  %s\n",
                      g_test_angles_deg[index], distance_mm,
                      status_name(status));
        }
    }
}