/**
 * @file    drv_rp2040.c
 * @brief   RP2040 timer, delay and PWM-reset helpers.  See drv_rp2040.h.
 */

#include "subsystem5.h"
#include "drv_rp2040.h"

#define WATCHDOG_TICK_ADDR \
    (0x40058000UL + 0x2CUL)              /* watchdog block + TICK offset */
#define WATCHDOG_TICK_ENABLE  (1UL << 9) /* bit 9: turn the tick on      */
#define CLK_REF_CYCLES_PER_US (12UL)     /* 12 MHz crystal / 12 = 1 MHz  */

EXPORT void rp2040_timer_start(void)
{
    /* Writing enable bit to watchdog */
    out_w(WATCHDOG_TICK_ADDR, WATCHDOG_TICK_ENABLE | CLK_REF_CYCLES_PER_US);
}

EXPORT UW rp2040_time_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

/* Busy-wait 'us' microseconds. Safe across TIMERAWL wrap-around. */
EXPORT void rp2040_delay_us(UW us)
{
    UW start = in_w(TIMER_TIMERAWL);

    while ((in_w(TIMER_TIMERAWL) - start) < us)
    {
        /* spin: do nothing until enough time has passed */
    }
}

EXPORT void rp2040_pwm_unreset(void)
{
    clr_w(RESETS_RESET, RESETS_RESET_PWM); /* release the reset */
    /* wait until it's ready */
    while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* hardware finishes within a few clock cycles */
    }
}