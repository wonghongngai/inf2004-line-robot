#include "subsystem5.h"

#define US_TRIG_PIN (6U) /* Temporary, to be changed to 16 */
#define US_ECHO_PIN (7U) /* Temporary, to be changed to 17 */

#define ECHO_TIMEOUT_US    (30000UL) /* 30 ms: longer than a 4 m round trip */
#define SOUND_MM_PER_MS    (343UL) /* speed of sound: 343 m/s = 343 mm per ms */
#define ROUND_TRIP_DIVISOR (2000UL) /* 1000 us per ms, x2 for there-and-back */

#define WATCHDOG_TICK_ADDR \
    (0x40058000UL + 0x2CUL)              /* watchdog block + TICK offset */
#define WATCHDOG_TICK_ENABLE  (1UL << 9) /* bit 9: turn the tick on      */
#define CLK_REF_CYCLES_PER_US (12UL)     /* 12 MHz crystal / 12 = 1 MHz  */

clr_w(RESETS_RESET, RESETS_RESET_PWM); /* release the reset */
while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
{
} /* wait until it's ready */

/* Busy-wait 'us' microseconds. Safe across TIMERAWL wrap-around. */
static void delay_us(UW us)
{
    UW start = in_w(TIMER_TIMERAWL);

    while ((in_w(TIMER_TIMERAWL) - start) < us)
    {
        /* spin: do nothing until enough time has passed */
    }
}

/*
 * Wait while US_ECHO_PIN reads 'level'.
 * Returns TRUE if the pin changed in time, FALSE on timeout.
 * On TRUE, *p_when holds the time (TIMERAWL, in us) the change was seen.
 */
static BOOL echo_wait_while(UINT level, UW timeout_us, UW * p_when)
{
    UW start = in_w(TIMER_TIMERAWL);
    UW now;

    for (;;)
    {
        now = in_w(TIMER_TIMERAWL);

        if (level != gpio_get_val(US_ECHO_PIN))
        {
            *p_when = now;
            return TRUE;
        }
        if ((now - start) > timeout_us)
        {
            return FALSE;
        }
    }
}

EXPORT void obstacle_task(INT stacd, void * exinf)
{
    /* Setting the pins */
    gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_pin(US_ECHO_PIN, GPIO_MODE_IN);
    gpio_set_val(US_TRIG_PIN, 0);

    /* To hide unused warnings */
    (void)stacd;
    (void)exinf;

    /* Writing enable bit to watchdog */
    out_w(WATCHDOG_TICK_ADDR, WATCHDOG_TICK_ENABLE | CLK_REF_CYCLES_PER_US);

    for (;;)
    {
        /* Send trigger HIGH for ultrasonic sensor */
        gpio_set_val(US_TRIG_PIN, 1);
        delay_us(12); /* 12 gives some margin over 10us */
        gpio_set_val(US_TRIG_PIN, 0);

        UW echo_start;
        UW echo_end;

        if (!echo_wait_while(0U, ECHO_TIMEOUT_US, &echo_start))
        {
            tm_printf((UB *)"S5: no echo (never started)\n");
        }
        else if (!echo_wait_while(1U, ECHO_TIMEOUT_US, &echo_end))
        {
            tm_printf((UB *)"S5: no echo (never ended)\n");
        }
        else
        {
            UW echo_us     = echo_end - echo_start;
            UW distance_mm = (echo_us * SOUND_MM_PER_MS) / ROUND_TRIP_DIVISOR;

            tm_printf((UB *)"S5: echo=%u us  d=%u mm\n", echo_us, distance_mm);
        }
        tk_dly_tsk(500);
    }
}