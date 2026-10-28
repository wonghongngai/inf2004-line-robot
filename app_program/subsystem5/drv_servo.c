/**
 * @file    drv_servo.c
 * @brief   SG90 servo driver.  See drv_servo.h.
 */

#include "subsystem5.h"
#include "drv_rp2040.h"
#include "drv_servo.h"

#define SERVO_PIN (14U) /* even pin = channel A */
#if ((SERVO_PIN & 1U) != 0U)
#error "SERVO_PIN must be even (PWM channel A)"
#endif

#define PWM_DIV_INT_SHIFT (4U) /* DIV integer part is bits 11:4  */

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
#define SERVO_CAL_SPAN_DEG (90UL) /* calibrated across 45 R .. 45 L */

/*
 * Robot angle (90 = forward, >90 = left) -> pulse width in us.
 * Clamped to SERVO_ANGLE_MIN/MAX so the horn never reaches its end stops.
 */
EXPORT UW servo_angle_to_pulse_us(UW angle_deg)
{
    if (angle_deg < SERVO_ANGLE_MIN_DEG)
    {
        angle_deg = SERVO_ANGLE_MIN_DEG;
    }
    if (angle_deg > SERVO_ANGLE_MAX_DEG)
    {
        angle_deg = SERVO_ANGLE_MAX_DEG;
    }

    /* Adding half the divisor rounds to the nearest us.  Never negative. */
    return SERVO_US_AT_0_DEG +
           (((angle_deg * SERVO_US_PER_90_DEG) + (SERVO_CAL_SPAN_DEG / 2UL)) /
            SERVO_CAL_SPAN_DEG);
}

EXPORT void servo_set_angle(UW angle_deg)
{
    pwm_set_cc(SERVO_PIN, servo_angle_to_pulse_us(angle_deg));
}

EXPORT void servo_init(void)
{
    rp2040_timer_start();
    rp2040_pwm_unreset();

    /* Servo: slice in normal free-running mode (DIVMODE = 0), enable off */
    out_w(SERVO_CSR_ADDR, 0U);
    out_w(SERVO_DIV_ADDR, SERVO_CLKDIV_INT << PWM_DIV_INT_SHIFT);
    pwm_set_wrap(SERVO_PIN, SERVO_TOP);

    /* Load a pulse BEFORE enabling, so servo's first pulse is safe */
    pwm_set_cc(SERVO_PIN, servo_angle_to_pulse_us(SERVO_ANGLE_CENTRE_DEG));
    pwm_set_pin(SERVO_PIN);
    pwm_set_enabled(SERVO_PIN, TRUE);
}