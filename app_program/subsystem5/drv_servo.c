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

/* Speed: 0.1 s per 60 deg, plus wobble margin based on spec sheet */
#define SERVO_MS_PER_60_DEG    (100UL)
#define SERVO_SETTLE_MARGIN_MS (20UL)

#define SERVO_CAL_SPAN_DDEG (SERVO_CAL_SPAN_DEG * SERVO_DDEG_PER_DEG)
#define SERVO_MIN_DDEG      (SERVO_ANGLE_MIN_DEG * SERVO_DDEG_PER_DEG)
#define SERVO_MAX_DDEG      (SERVO_ANGLE_MAX_DEG * SERVO_DDEG_PER_DEG)

/* Last commanded angle in ddeg (not measured: the servo can't report it) */
static UW g_angle_ddeg = SERVO_ANGLE_CENTRE_DEG * SERVO_DDEG_PER_DEG;

/* Limit an angle to the servo's safe range */
static UW servo_clamp_deg(UW angle_deg)
{
    if (angle_deg < SERVO_ANGLE_MIN_DEG)
    {
        return SERVO_ANGLE_MIN_DEG;
    }
    if (angle_deg > SERVO_ANGLE_MAX_DEG)
    {
        return SERVO_ANGLE_MAX_DEG;
    }
    return angle_deg;
}

/* Limit an angle in ddeg to the servo's safe range */
static UW servo_clamp_ddeg(UW angle_ddeg)
{
    if (angle_ddeg < SERVO_MIN_DDEG)
    {
        return SERVO_MIN_DDEG;
    }
    if (angle_ddeg > SERVO_MAX_DDEG)
    {
        return SERVO_MAX_DDEG;
    }
    return angle_ddeg;
}

/* Distance between two angles (unsigned, either order) */
static UW servo_abs_diff(UW a, UW b)
{
    return (a > b) ? (a - b) : (b - a);
}

/*
 * Robot angle in ddeg (900 = forward, > 900 = left) -> pulse width in us.
 * Clamped to SERVO_ANGLE_MIN/MAX so the horn never reaches its end stops.
 */
EXPORT UW servo_ddeg_to_pulse_us(UW angle_ddeg)
{
    angle_ddeg = servo_clamp_ddeg(angle_ddeg);

    /* Adding half the divisor rounds to the nearest us.  Never negative. */
    return SERVO_US_AT_0_DEG +
           (((angle_ddeg * SERVO_US_PER_90_DEG) + (SERVO_CAL_SPAN_DDEG / 2UL)) /
            SERVO_CAL_SPAN_DDEG);
}

/* Robot angle in whole degrees -> pulse width in us (S5-U11) */
EXPORT UW servo_angle_to_pulse_us(UW angle_deg)
{
    return servo_ddeg_to_pulse_us(servo_clamp_deg(angle_deg) *
                                  SERVO_DDEG_PER_DEG);
}

/* Command the servo to an angle in ddeg, at once */
EXPORT void servo_set_angle_ddeg(UW angle_ddeg)
{
    g_angle_ddeg = servo_clamp_ddeg(angle_ddeg);
    pwm_set_cc(SERVO_PIN, servo_ddeg_to_pulse_us(g_angle_ddeg));
}

/* Command the servo to an angle in whole degrees, at once */
EXPORT void servo_set_angle(UW angle_deg)
{
    servo_set_angle_ddeg(servo_clamp_deg(angle_deg) * SERVO_DDEG_PER_DEG);
}

/* Last commanded angle, in ddeg */
EXPORT UW servo_get_angle_ddeg(void)
{
    return g_angle_ddeg;
}

/* ---- Time-based glide profile (pure) -----------------------------------
 * Worked in "milli-ddeg" (1/1000 ddeg) so speeds in ddeg/s are simply
 * milli-ddeg per ms and no rounding error builds up during a move. */
#define MILLI_PER_DDEG (1000UL)
#define GLIDE_V0       (SERVO_GLIDE_START_DDEG_S)  /* milli-ddeg per ms */
#define GLIDE_V1       (SERVO_GLIDE_CRUISE_DDEG_S) /* milli-ddeg per ms */
#define GLIDE_RAMP_MS  (SERVO_GLIDE_RAMP_MS)
/* Distance covered while ramping V0 -> V1 (70 000 = 7 deg) */
#define GLIDE_RAMP_MDDEG          \
    ((GLIDE_V0 * GLIDE_RAMP_MS) + \
     (((GLIDE_V1 - GLIDE_V0) * GLIDE_RAMP_MS) / 2UL))

/* Distance covered 't_ms' into a ramp-up (t_ms <= GLIDE_RAMP_MS) */
static UW glide_ramp_mddeg(UW t_ms)
{
    return (GLIDE_V0 * t_ms) +
           (((GLIDE_V1 - GLIDE_V0) * t_ms * t_ms) / (2UL * GLIDE_RAMP_MS));
}

/* TRUE if the move is long enough to ramp up to cruise and back down */
static BOOL glide_is_long(UW distance_ddeg)
{
    return ((distance_ddeg * MILLI_PER_DDEG) >= (2UL * GLIDE_RAMP_MDDEG))
               ? TRUE
               : FALSE;
}

/* How long a glide takes, in ms */
EXPORT UW servo_profile_total_ms(UW distance_ddeg)
{
    UW distance_mddeg = distance_ddeg * MILLI_PER_DDEG;

    if (glide_is_long(distance_ddeg))
    {
        /* ramp + cruise (rounded up) + ramp */
        return (2UL * GLIDE_RAMP_MS) +
               ((distance_mddeg - (2UL * GLIDE_RAMP_MDDEG) + (GLIDE_V1 - 1UL)) /
                GLIDE_V1);
    }
    /* Short move: START speed throughout (still above the dead band) */
    return (distance_mddeg + (GLIDE_V0 - 1UL)) / GLIDE_V0;
}

/* How far a glide has got after 'elapsed_ms', in ddeg */
EXPORT UW servo_profile_ddeg(UW distance_ddeg, UW elapsed_ms)
{
    UW distance_mddeg = distance_ddeg * MILLI_PER_DDEG;
    UW total_ms       = servo_profile_total_ms(distance_ddeg);
    UW cruise_ms;
    UW done_mddeg;

    if (elapsed_ms >= total_ms)
    {
        return distance_ddeg;
    }

    if (glide_is_long(distance_ddeg))
    {
        cruise_ms = total_ms - (2UL * GLIDE_RAMP_MS);
        if (elapsed_ms <= GLIDE_RAMP_MS)
        {
            done_mddeg = glide_ramp_mddeg(elapsed_ms); /* speeding up */
        }
        else if (elapsed_ms <= (GLIDE_RAMP_MS + cruise_ms))
        {
            done_mddeg = GLIDE_RAMP_MDDEG +
                         (GLIDE_V1 * (elapsed_ms - GLIDE_RAMP_MS)); /* cruise */
            /* cruise time was rounded up: never run into the slow-down */
            if (done_mddeg > (distance_mddeg - GLIDE_RAMP_MDDEG))
            {
                done_mddeg = distance_mddeg - GLIDE_RAMP_MDDEG;
            }
        }
        else
        {
            /* slowing down: the ramp-up mirrored from the end */
            done_mddeg =
                distance_mddeg - glide_ramp_mddeg(total_ms - elapsed_ms);
        }
    }
    else
    {
        done_mddeg = GLIDE_V0 * elapsed_ms;
    }

    if (done_mddeg > distance_mddeg)
    {
        done_mddeg = distance_mddeg;
    }
    return (done_mddeg + (MILLI_PER_DDEG / 2UL)) / MILLI_PER_DDEG;
}

/* Current glide (only obstacle_task moves the servo, so no locking) */
static UW g_glide_start_ddeg  = SERVO_ANGLE_CENTRE_DEG * SERVO_DDEG_PER_DEG;
static UW g_glide_target_ddeg = SERVO_ANGLE_CENTRE_DEG * SERVO_DDEG_PER_DEG;
static UW g_glide_t0_us       = 0U;

/* Start a glide from where the servo is now */
EXPORT void servo_glide_begin(UW angle_deg)
{
    g_glide_start_ddeg  = g_angle_ddeg;
    g_glide_target_ddeg = servo_clamp_deg(angle_deg) * SERVO_DDEG_PER_DEG;
    g_glide_t0_us       = rp2040_time_us();
}

/* Output where the glide should be now; TRUE once it has arrived */
EXPORT BOOL servo_glide_update(void)
{
    UW distance_ddeg;
    UW done_ddeg;
    UW elapsed_ms;

    if (g_angle_ddeg == g_glide_target_ddeg)
    {
        return TRUE;
    }

    /* Unsigned subtraction is safe across TIMERAWL wrap-around */
    elapsed_ms    = (rp2040_time_us() - g_glide_t0_us) / 1000U;
    distance_ddeg = servo_abs_diff(g_glide_start_ddeg, g_glide_target_ddeg);
    done_ddeg     = servo_profile_ddeg(distance_ddeg, elapsed_ms);

    servo_set_angle_ddeg((g_glide_target_ddeg > g_glide_start_ddeg)
                             ? (g_glide_start_ddeg + done_ddeg)
                             : (g_glide_start_ddeg - done_ddeg));

    return (g_angle_ddeg == g_glide_target_ddeg) ? TRUE : FALSE;
}

/* Glide to an angle (blocking), then let the horn settle */
EXPORT void servo_glide_to(UW angle_deg)
{
    servo_glide_begin(angle_deg);
    if (g_angle_ddeg == g_glide_target_ddeg)
    {
        return; /* not moving: nothing to wait for */
    }

    while (!servo_glide_update())
    {
        tk_dly_tsk(SERVO_GLIDE_POLL_MS);
    }
    tk_dly_tsk(SERVO_GLIDE_SETTLE_MS);
}

/* Work out how long the servo needs to reach a new angle */
EXPORT UW servo_settle_ms(UW from_deg, UW to_deg)
{
    UW delta_deg =
        (from_deg > to_deg) ? (from_deg - to_deg) : (to_deg - from_deg);

    if (0U == delta_deg)
    {
        return 0U; /* not moving: nothing to wait for */
    }

    /* (x + 59) / 60 rounds UP: never wait too little */
    return (((delta_deg * SERVO_MS_PER_60_DEG) + 59UL) / 60UL) +
           SERVO_SETTLE_MARGIN_MS;
}

/* Move the servo and wait until it has arrived: now a smooth glide (the
 * old full-speed jump made the scan jerky and shook the robot) */
EXPORT void servo_move_and_settle(UW angle_deg)
{
    servo_glide_to(angle_deg);
}

/* Jump straight to an angle at full speed, then wait for the horn */
EXPORT void servo_jump_to(UW angle_deg)
{
    UW from_deg = g_angle_ddeg / SERVO_DDEG_PER_DEG;
    UW wait_ms;

    servo_set_angle(angle_deg);

    wait_ms = servo_settle_ms(from_deg, g_angle_ddeg / SERVO_DDEG_PER_DEG);
    if (wait_ms > 0U)
    {
        tk_dly_tsk(wait_ms);
    }
}

/* Initialise the servo */
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

    g_angle_ddeg = SERVO_ANGLE_CENTRE_DEG * SERVO_DDEG_PER_DEG;

    /* We don't know where the horn was at power-up: allow a full swing */
    tk_dly_tsk(servo_settle_ms(SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG));
}