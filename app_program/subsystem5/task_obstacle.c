#include "subsystem5.h"
#include "drv_rp2040.h"
#include "drv_ultrasonic.h"

#define SERVO_PIN (14U) /* even pin = channel A */
#if ((SERVO_PIN & 1U) != 0U)
#error "SERVO_PIN must be even (PWM channel A)"
#endif

#define PWM_DIV_INT_SHIFT (4U) /* DIV integer part is bits 11:4  */

/* timing */
#define PING_PERIOD_MS \
    (500U) /* >= 60 ms between pings; 500 keeps log readable */

#define SERVO_SLICE      ((SERVO_PIN >> 1U) & 0x7U)
#define SERVO_SLICE_BASE (PWM_BASE + (0x14UL * SERVO_SLICE))
#define SERVO_CSR_ADDR   (SERVO_SLICE_BASE + PWM_CHx_CSR)
#define SERVO_DIV_ADDR   (SERVO_SLICE_BASE + PWM_CHx_DIV)

#define SERVO_CLKDIV_INT (125UL)   /* 125 MHz / 125 = 1 count per us */
#define SERVO_PERIOD_US  (20000UL) /* 20 ms = 50 Hz */
#define SERVO_TOP        (SERVO_PERIOD_US - 1UL) /* counts 0..19999 */

/* Calibration, measured with a folded A4 corner (Step 7c) */
#define SERVO_US_PER_90_DEG (1020UL) /* 2040 us (45 L) - 1020 us (45 R) */
#define SERVO_US_AT_0_DEG \
    (510UL) /* 1530 - 1020: 90 deg = 1530 us, left = more */
#define SERVO_ANGLE_MIN_DEG (20U) /* safety limits: sweep needs 30-150 */
#define SERVO_ANGLE_MAX_DEG (160U)

/*
 * Robot angle (90 = forward, >90 = left) -> pulse width in us.
 * Clamped to SERVO_ANGLE_MIN/MAX so the horn never reaches its end stops.
 */
static UW servo_angle_to_pulse_us(UW angle_deg)
{
    if (angle_deg < SERVO_ANGLE_MIN_DEG)
    {
        angle_deg = SERVO_ANGLE_MIN_DEG;
    }
    if (angle_deg > SERVO_ANGLE_MAX_DEG)
    {
        angle_deg = SERVO_ANGLE_MAX_DEG;
    }

    /* Rounded, and never negative: see Step 7d */
    return SERVO_US_AT_0_DEG +
           (((angle_deg * SERVO_US_PER_90_DEG) + 45UL) / 90UL);
}

static void servo_set_angle(UW angle_deg)
{
    pwm_set_cc(SERVO_PIN, servo_angle_to_pulse_us(angle_deg));
}

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
    UW                  pulse_us;
    UW                  distance_mm;
    ultrasonic_status_t status;

    /* To hide unused warnings */
    (void)stacd;
    (void)exinf;

    ultrasonic_init();

    rp2040_timer_start();
    rp2040_pwm_unreset();

    /* Servo: slice in normal free-running mode (DIVMODE = 0), enable off */
    out_w(SERVO_CSR_ADDR, 0U);
    out_w(SERVO_DIV_ADDR, SERVO_CLKDIV_INT << PWM_DIV_INT_SHIFT);
    pwm_set_wrap(SERVO_PIN, SERVO_TOP);

    /* Load a pulse BEFORE enabling, so servo's first pulse is safe */
    pwm_set_cc(SERVO_PIN, servo_angle_to_pulse_us(90U));
    pwm_set_pin(SERVO_PIN);
    pwm_set_enabled(SERVO_PIN, TRUE);

    /* A task must never return: loop forever */
    for (;;)
    {
        for (index = 0U; index < TEST_ANGLE_COUNT; index++)
        {
            pulse_us = servo_angle_to_pulse_us(g_test_angles_deg[index]);
            pwm_set_cc(SERVO_PIN, pulse_us);
            tk_dly_tsk(1000); /* let the servo arrive before pinging */

            status = ultrasonic_measure_mm(&distance_mm);
            tm_printf((UB *)"S5: angle %u (%u us)  d=%u mm  %s\n",
                      g_test_angles_deg[index], pulse_us, distance_mm,
                      status_name(status));
        }
    }
}