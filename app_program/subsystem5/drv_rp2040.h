/**
 * @file    drv_rp2040.h
 * @brief   Small RP2040 helpers for subsystem 5: the 1 MHz timer, short
 *          busy-wait delays, and releasing the PWM block from reset.
 *
 * Why: this kernel port does not set up the watchdog tick that drives
 * the TIMER at 1 MHz, and leaves the PWM block held in reset.
 */

#ifndef DRV_RP2040_H
#define DRV_RP2040_H

#include <tk/tkernel.h>

/** Start the 1 us tick that drives TIMERAWL.  Call once at startup. */
IMPORT void rp2040_timer_start(void);

/** @return Microseconds since the timer started (wraps every ~71 min). */
IMPORT UW rp2040_time_us(void);

/** Busy-wait 'us' microseconds.  Only for short waits (e.g. 12 us). */
IMPORT void rp2040_delay_us(UW us);

/** Take the PWM block out of reset.  Safe to call more than once. */
IMPORT void rp2040_pwm_unreset(void);

#endif /* DRV_RP2040_H */