/**
 * @file    drv_ultrasonic.h
 * @brief   HC-SR04P ultrasonic driver.  The echo is timed in hardware by a
 *          PWM slice ("count while B is high"), so the caller's task sleeps
 *          during the measurement and timing is unaffected by other tasks.
 *
 * Only obstacle_task may call these functions (it owns the sensor).
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <tk/tkernel.h> /* UW */

/* Valid range 2cm - 400cm */
#define ULTRASONIC_MIN_MM (20U)
#define ULTRASONIC_MAX_MM (4000U)

typedef enum
{
    ULTRASONIC_OK = 0,   /* *p_distance_mm is valid                 */
    ULTRASONIC_BUSY,     /* ECHO still high from the last ping      */
    ULTRASONIC_NO_ECHO,  /* nothing came back inside the window     */
    ULTRASONIC_TOO_NEAR, /* closer than ULTRASONIC_MIN_MM: invalid  */
    ULTRASONIC_TOO_FAR   /* further than ULTRASONIC_MAX_MM: invalid */
} ultrasonic_status_t;

/** Set up the TRIG pin and the echo-timing PWM slice.  Call once. */
IMPORT void ultrasonic_init(void);

/**
 * Fire one ping.  Blocks the calling task for ~30-40 ms (it sleeps).
 * @param  p_distance_mm  Out: distance in mm, or 0 if not ULTRASONIC_OK.
 * @return What happened.  Only ULTRASONIC_OK readings may be used.
 * Leave at least 60 ms between calls (sensor requirement).
 */
IMPORT ultrasonic_status_t ultrasonic_measure_mm(UW * p_distance_mm);

#endif /* DRV_ULTRASONIC_H */