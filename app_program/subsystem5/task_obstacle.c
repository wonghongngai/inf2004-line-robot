#include "subsystem5.h"
#include "drv_rp2040.h"

#define US_TRIG_PIN (6U)  /* Temporary, to be changed to 16 */
#define US_ECHO_PIN (7U)  /* Temporary, to be changed to 17 */
#define SERVO_PIN   (14U) /* even pin = channel A */
#if ((US_ECHO_PIN & 1U) == 0U)
#error "US_ECHO_PIN must be odd (PWM channel B)"
#endif
#if ((SERVO_PIN & 1U) != 0U)
#error "SERVO_PIN must be even (PWM channel A)"
#endif

#define SOUND_MM_PER_MS    (343UL) /* speed of sound: 343 m/s = 343 mm per ms */
#define ROUND_TRIP_DIVISOR (2000UL) /* 1000 us per ms, x2 for there-and-back */

/* PWM slice that times the echo -> slice (N >> 1) & 7 */
#define ECHO_SLICE      ((US_ECHO_PIN >> 1U) & 0x7U)
#define ECHO_SLICE_BASE (PWM_BASE + (0x14UL * ECHO_SLICE))
#define ECHO_CSR_ADDR   (ECHO_SLICE_BASE + PWM_CHx_CSR)
#define ECHO_DIV_ADDR   (ECHO_SLICE_BASE + PWM_CHx_DIV)
#define ECHO_CTR_ADDR   (ECHO_SLICE_BASE + PWM_CHx_CTR)
#define ECHO_TOP_ADDR   (ECHO_SLICE_BASE + PWM_CHx_TOP)

/* Register field values */
#define PWM_CSR_DIVMODE_LEVEL (1UL << 4) /* count only while pin B is high */
#define PWM_DIV_INT_SHIFT     (4U)       /* DIV integer part is bits 11:4  */
#define ECHO_CLKDIV_INT       (125UL)    /* 125 MHz / 125 = 1 count per us */
#define ECHO_CTR_MAX          (0xFFFFUL) /* 16-bit counter: 65 535 us max  */

/* timing */
#define ECHO_WINDOW_MS (30U) /* sleep while hardware times echo (4 m = 23 ms) */
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

/* Fire one ping and print results. Blocks for 30ms - 40ms */
static void ultrasonic_ping_and_print(void)
{
    UW   echo_us;
    UW   distance_mm;
    BOOL b_still_high;

    if (0U != gpio_get_val(US_ECHO_PIN))
    {
        tm_printf((UB *)"S5: busy (echo still high)\n");
    }
    else
    {
        /* Zero the stopwatch and arm it */
        out_w(ECHO_CTR_ADDR, 0U);
        set_w(ECHO_CSR_ADDR, PWM_CH_CSR_EN);

        /* Send trigger HIGH for ultrasonic sensor */
        gpio_set_val(US_TRIG_PIN, 1);
        rp2040_delay_us(12U); /* 12 gives some margin over 10us */
        gpio_set_val(US_TRIG_PIN, 0);

        /* Sleep; the hardware counts only while ECHO is high */
        tk_dly_tsk(ECHO_WINDOW_MS);

        b_still_high = (0U != gpio_get_val(US_ECHO_PIN));
        echo_us      = in_w(ECHO_CTR_ADDR) & ECHO_CTR_MAX;
        clr_w(ECHO_CSR_ADDR, PWM_CH_CSR_EN);

        /* Interpret */
        if (b_still_high)
        {
            tm_printf((UB *)"S5: no echo (too long)\n");
        }
        else if (0U == echo_us)
        {
            tm_printf((UB *)"S5: no echo\n");
        }
        else
        {
            distance_mm = (echo_us * SOUND_MM_PER_MS) / ROUND_TRIP_DIVISOR;
            tm_printf((UB *)"S5: echo=%u us  d=%u mm\n", echo_us, distance_mm);
        }
    }
}

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

EXPORT void obstacle_task(INT stacd, void * exinf)
{
    UW index;
    UW pulse_us;

    /* Setting the pins */
    gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(US_TRIG_PIN, 0);

    /* To hide unused warnings */
    (void)stacd;
    (void)exinf;

    rp2040_timer_start();
    rp2040_pwm_unreset();

    out_w(ECHO_CSR_ADDR, PWM_CSR_DIVMODE_LEVEL);
    out_w(ECHO_DIV_ADDR, ECHO_CLKDIV_INT << PWM_DIV_INT_SHIFT);
    out_w(ECHO_TOP_ADDR, ECHO_CTR_MAX);

    /* Give the ECHO pin to the PWM block (also enables its input) */
    pwm_set_pin(US_ECHO_PIN);

    /* Safety: the sensor drives ECHO, so the Pico must never drive it */
    set_w(GPIO(US_ECHO_PIN), GPIO_OD);

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
            tm_printf((UB *)"S5: angle %u -> %u us\n", g_test_angles_deg[index],
                      pulse_us);
            tk_dly_tsk(1000);
        }
    }
}