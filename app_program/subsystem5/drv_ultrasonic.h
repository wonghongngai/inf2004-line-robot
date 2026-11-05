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

/* How long to listen for the echo (rounded up to the 10 ms tick).
 * Echo time = distance x 2 / 343 mm per ms. */
#define ULTRASONIC_WINDOW_FULL_MS (30U) /* up to 4 m (23 ms echo)          */
#define ULTRASONIC_WINDOW_SCAN_MS (20U) /* up to ~3 m: faster, for scans   */

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
 * Fire one ping.  Blocks the calling task for about window_ms + 10 ms
 * (it sleeps while the hardware times the echo).
 * @param  window_ms      ULTRASONIC_WINDOW_FULL_MS or _SCAN_MS.
 * @param  p_distance_mm  Out: distance in mm, or 0 if not ULTRASONIC_OK.
 * @return What happened.  Only ULTRASONIC_OK readings may be used.
 * Leave at least 60 ms between calls (sensor requirement).
 */
IMPORT ultrasonic_status_t ultrasonic_measure_mm(UW   window_ms,
                                                 UW * p_distance_mm);

/**
 * Non-blocking ping, part 1: arm the echo timer and fire the trigger
 * (busy-waits only the ~12 us trigger pulse).  The hardware times the
 * echo on its own, so the caller can keep working (e.g. move the servo)
 * and call ultrasonic_ping_finish() once the window has passed.
 * @return FALSE (nothing fired) if ECHO is still high from the last ping.
 */
IMPORT BOOL ultrasonic_ping_start(void);

/**
 * Non-blocking ping, part 2: read and convert the echo time.  Call at
 * least the listening window (e.g. ULTRASONIC_WINDOW_SCAN_MS) after
 * ultrasonic_ping_start(); later is fine.
 * @param  p_distance_mm  Out: distance in mm, or 0 if not ULTRASONIC_OK.
 * @return What happened (NO_ECHO if no ping was started).
 */
IMPORT ultrasonic_status_t ultrasonic_ping_finish(UW * p_distance_mm);
#endif /* DRV_ULTRASONIC_H */