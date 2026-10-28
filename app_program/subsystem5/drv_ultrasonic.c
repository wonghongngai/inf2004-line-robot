/**
 * @file    drv_ultrasonic.c
 * @brief   HC-SR04P driver.  See drv_ultrasonic.h.
 */

#include "subsystem5.h"
#include "drv_rp2040.h"
#include "drv_ultrasonic.h"

#define US_TRIG_PIN (6U) /* Temporary, to be changed to 16 */
#define US_ECHO_PIN (7U) /* Temporary, to be changed to 17 */

#if ((US_ECHO_PIN & 1U) == 0U)
#error "US_ECHO_PIN must be odd (PWM channel B)"
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
#define ECHO_CLKDIV_INT      (125UL)    /* 125 MHz / 125 = 1 count per us */
#define ECHO_CTR_MAX          (0xFFFFUL) /* 16-bit counter: 65 535 us max  */


#define TRIG_PULSE_US (12U) /* datasheet needs >= 10 us; 12 gives margin */

EXPORT void ultrasonic_init(void)
{
    rp2040_timer_start();
    rp2040_pwm_unreset();

    /* Setting the pins */
    gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_val(US_TRIG_PIN, 0);

    out_w(ECHO_CSR_ADDR, PWM_CSR_DIVMODE_LEVEL);
    out_w(ECHO_DIV_ADDR, ECHO_CLKDIV_INT << PWM_DIV_INT_SHIFT);
    out_w(ECHO_TOP_ADDR, ECHO_CTR_MAX);

    /* Give the ECHO pin to the PWM block (also enables its input) */
    pwm_set_pin(US_ECHO_PIN);

    /* Safety: the sensor drives ECHO, so the Pico must never drive it */
    set_w(GPIO(US_ECHO_PIN), GPIO_OD);
}

EXPORT ultrasonic_status_t ultrasonic_measure_mm(UW   window_ms,
                                                 UW * p_distance_mm)
{
    UW   echo_us;
    UW   distance_mm;
    BOOL b_still_high;

    /* never write through a NULL pointer */
    if (NULL == p_distance_mm)
    {
        return ULTRASONIC_NO_ECHO;
    }

    *p_distance_mm = 0U;

    if (0U != gpio_get_val(US_ECHO_PIN))
    {
        return ULTRASONIC_BUSY;
    }

    /* Zero the stopwatch and arm it */
    out_w(ECHO_CTR_ADDR, 0U);
    set_w(ECHO_CSR_ADDR, PWM_CH_CSR_EN);

    /* Send trigger HIGH for ultrasonic sensor */
    gpio_set_val(US_TRIG_PIN, 1);
    rp2040_delay_us(TRIG_PULSE_US);
    gpio_set_val(US_TRIG_PIN, 0);

    /* Sleep; the hardware counts only while ECHO is high */
    tk_dly_tsk(window_ms);

    b_still_high = (0U != gpio_get_val(US_ECHO_PIN));
    echo_us      = in_w(ECHO_CTR_ADDR) & ECHO_CTR_MAX;
    clr_w(ECHO_CSR_ADDR, PWM_CH_CSR_EN);

    /* turn the raw result into a status, no printing */
    if (b_still_high || (0U == echo_us))
    {
        return ULTRASONIC_NO_ECHO;
    }

    distance_mm = (echo_us * SOUND_MM_PER_MS) / ROUND_TRIP_DIVISOR;

    if (distance_mm < ULTRASONIC_MIN_MM)
    {
        return ULTRASONIC_TOO_NEAR;
    }
    if (distance_mm > ULTRASONIC_MAX_MM)
    {
        return ULTRASONIC_TOO_FAR;
    }

    *p_distance_mm = distance_mm;
    return ULTRASONIC_OK;
}