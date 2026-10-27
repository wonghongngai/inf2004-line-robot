#include "subsystem5.h"

#define US_TRIG_PIN (6U) /* Temporary, to be changed to 16 */
#define US_ECHO_PIN (7U) /* Temporary, to be changed to 17 */

#define SOUND_MM_PER_MS    (343UL) /* speed of sound: 343 m/s = 343 mm per ms */
#define ROUND_TRIP_DIVISOR (2000UL) /* 1000 us per ms, x2 for there-and-back */

#define WATCHDOG_TICK_ADDR \
    (0x40058000UL + 0x2CUL)              /* watchdog block + TICK offset */
#define WATCHDOG_TICK_ENABLE  (1UL << 9) /* bit 9: turn the tick on      */
#define CLK_REF_CYCLES_PER_US (12UL)     /* 12 MHz crystal / 12 = 1 MHz  */

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

/* Busy-wait 'us' microseconds. Safe across TIMERAWL wrap-around. */
static void delay_us(UW us)
{
    UW start = in_w(TIMER_TIMERAWL);

    while ((in_w(TIMER_TIMERAWL) - start) < us)
    {
        /* spin: do nothing until enough time has passed */
    }
}

EXPORT void obstacle_task(INT stacd, void * exinf)
{
    UW   echo_us;
    UW   distance_mm;
    BOOL b_still_high;

    /* Setting the pins */
    gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(US_TRIG_PIN, 0);

    /* To hide unused warnings */
    (void)stacd;
    (void)exinf;

    /* Writing enable bit to watchdog */
    out_w(WATCHDOG_TICK_ADDR, WATCHDOG_TICK_ENABLE | CLK_REF_CYCLES_PER_US);

    clr_w(RESETS_RESET, RESETS_RESET_PWM); /* release the reset */
    /* wait until it's ready */
    while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* hardware finishes within a few clock cycles */
    }

    out_w(ECHO_CSR_ADDR, PWM_CSR_DIVMODE_LEVEL);
    out_w(ECHO_DIV_ADDR, ECHO_CLKDIV_INT << PWM_DIV_INT_SHIFT);
    out_w(ECHO_TOP_ADDR, ECHO_CTR_MAX);

    /* Give the ECHO pin to the PWM block (also enables its input) */
    pwm_set_pin(US_ECHO_PIN);

    /* Safety: the sensor drives ECHO, so the Pico must never drive it */
    set_w(GPIO(US_ECHO_PIN), GPIO_OD);

    for (;;)
    {
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
            delay_us(12); /* 12 gives some margin over 10us */
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
                tm_printf((UB *)"S5: echo=%u us  d=%u mm\n", echo_us,
                          distance_mm);
            }
        }

        /* Wait before the next ping */
        tk_dly_tsk(PING_PERIOD_MS);
    }
}