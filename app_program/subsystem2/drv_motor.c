/*
 * drv_motor.c
 * Buddy 2 -- sole low-level owner of the Robo Pico motor pins.
 *
 * Robo Pico mapping used by the team's design:
 *
 *   Motor 1:
 *      M1A = GP8
 *      M1B = GP9
 *
 *   Motor 2:
 *      M2A = GP10
 *      M2B = GP11
 *
 * This project has no backwards command.
 *
 * MxA is therefore currently treated as the forward PWM output while
 * MxB remains LOW.
 *
 * Actual motor direction depends on physical motor wiring.  If a motor
 * rotates backwards during hardware testing, swap the physical motor wires
 * or deliberately swap its forward/reverse software pins.
 */
#include "drv_motor.h"

#define MOTOR_PWM_REG(slice, offset) (PWM_BASE + (slice) * 0x14u + (offset))

#define MOTOR_LEFT_FORWARD_PIN    8U
#define MOTOR_LEFT_REVERSE_PIN    9U

#define MOTOR_RIGHT_FORWARD_PIN  10U
#define MOTOR_RIGHT_REVERSE_PIN  11U

/*
 * PWM counter top.
 *
 * 1000 gives convenient conversion between percentage and compare value.
 */
#define MOTOR_PWM_WRAP 1000U
#define MOTOR_PWM_FREQUENCY_HZ 10000U
/* RP2040 divider is 8.4 fixed point. Round to the nearest 1/16 step.
   At this port's 125 MHz SYSCLK this is 12.5, giving about 9.99 kHz. */
#define MOTOR_PWM_DIV_16 \
    ((SYSCLK * 1000000u * 16u + MOTOR_PWM_FREQUENCY_HZ * (MOTOR_PWM_WRAP + 1u) / 2u) / \
     (MOTOR_PWM_FREQUENCY_HZ * (MOTOR_PWM_WRAP + 1u)))

/*
 * Ensure requested PWM remains between 0% and 100%.
 */
static float clamp_percent(float percent)
{
    if (percent < 0.0f) {
        return 0.0f;
    }

    if (percent > 100.0f) {
        return 100.0f;
    }

    return percent;
}

/*
 * Convert percentage into PWM compare value.
 */
static UW percent_to_cc(float percent)
{
    return
        (UW)(
            (
                clamp_percent(percent) *
                (float)MOTOR_PWM_WRAP
            ) /
            100.0f
        );
}

/*
 * Configure one GPIO pin for PWM.
 */
static ER init_pwm_pin(UINT pin)
{
    ER er;

    er = pwm_set_pin(pin);

    if (er < E_OK) {
        return er;
    }

    /* Default divider=1 would produce ~125 kHz, above Robo Pico's 20 kHz
       rating. Use the 10 kHz frequency from Cytron's motor example. */
    out_w(MOTOR_PWM_REG((pin >> 1u) & 7u, PWM_CHx_DIV), MOTOR_PWM_DIV_16);

    er = pwm_set_wrap(
        pin,
        MOTOR_PWM_WRAP
    );

    if (er < E_OK) {
        return er;
    }

    /*
     * Start with zero motor output.
     */
    return pwm_set_cc(
        pin,
        0U
    );
}

ER drv_motor_init(void)
{
    ER er;

    /*
     * Configure left motor outputs.
     */
    er = init_pwm_pin(
        MOTOR_LEFT_FORWARD_PIN
    );

    if (er < E_OK) {
        return er;
    }

    er = init_pwm_pin(
        MOTOR_LEFT_REVERSE_PIN
    );

    if (er < E_OK) {
        return er;
    }

    /*
     * Configure right motor outputs.
     */
    er = init_pwm_pin(
        MOTOR_RIGHT_FORWARD_PIN
    );

    if (er < E_OK) {
        return er;
    }

    er = init_pwm_pin(
        MOTOR_RIGHT_REVERSE_PIN
    );

    if (er < E_OK) {
        return er;
    }

    /*
     * GP8 / GP9 share one PWM slice.
     *
     * GP10 / GP11 share another PWM slice.
     *
     * Therefore enabling one channel from each pair enables the
     * corresponding slice.
     */
    er = pwm_set_enabled(
        MOTOR_LEFT_FORWARD_PIN,
        TRUE
    );

    if (er < E_OK) {
        return er;
    }

    er = pwm_set_enabled(
        MOTOR_RIGHT_FORWARD_PIN,
        TRUE
    );

    if (er < E_OK) {
        return er;
    }

    /*
     * Always initialise in the safe stopped condition.
     */
    drv_motor_stop();

    return E_OK;
}

void drv_motor_set(
    float left_percent,
    float right_percent
)
{
    /*
     * Left motor forward PWM.
     */
    (void)pwm_set_cc(
        MOTOR_LEFT_FORWARD_PIN,
        percent_to_cc(left_percent)
    );

    /*
     * No backwards movement.
     */
    (void)pwm_set_cc(
        MOTOR_LEFT_REVERSE_PIN,
        0U
    );

    /*
     * Right motor forward PWM.
     */
    (void)pwm_set_cc(
        MOTOR_RIGHT_FORWARD_PIN,
        percent_to_cc(right_percent)
    );

    /*
     * No backwards movement.
     */
    (void)pwm_set_cc(
        MOTOR_RIGHT_REVERSE_PIN,
        0U
    );
}

void drv_motor_stop(void)
{
    /*
     * Set every motor-driver input to zero.
     */
    (void)pwm_set_cc(
        MOTOR_LEFT_FORWARD_PIN,
        0U
    );

    (void)pwm_set_cc(
        MOTOR_LEFT_REVERSE_PIN,
        0U
    );

    (void)pwm_set_cc(
        MOTOR_RIGHT_FORWARD_PIN,
        0U
    );

    (void)pwm_set_cc(
        MOTOR_RIGHT_REVERSE_PIN,
        0U
    );
}

ER drv_motor_diagnostics(MOTOR_DIAGNOSTICS *out)
{
    UW left_start, right_start;
    const UW left_slice = (MOTOR_LEFT_FORWARD_PIN >> 1u) & 7u;
    const UW right_slice = (MOTOR_RIGHT_FORWARD_PIN >> 1u) & 7u;
    if (out == 0) return E_PAR;
    out->reset_done = in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM;
    out->left_csr = in_w(MOTOR_PWM_REG(left_slice, PWM_CHx_CSR));
    out->right_csr = in_w(MOTOR_PWM_REG(right_slice, PWM_CHx_CSR));
    out->left_compare = in_w(MOTOR_PWM_REG(left_slice, PWM_CHx_CC));
    out->right_compare = in_w(MOTOR_PWM_REG(right_slice, PWM_CHx_CC));
    out->left_counting = out->right_counting = FALSE;
    left_start = in_w(MOTOR_PWM_REG(left_slice, PWM_CHx_CTR));
    right_start = in_w(MOTOR_PWM_REG(right_slice, PWM_CHx_CTR));
    for (unsigned i = 0; i < 64u; ++i) {
        if (in_w(MOTOR_PWM_REG(left_slice, PWM_CHx_CTR)) != left_start)
            out->left_counting = TRUE;
        if (in_w(MOTOR_PWM_REG(right_slice, PWM_CHx_CTR)) != right_start)
            out->right_counting = TRUE;
    }
    if (!out->reset_done || !out->left_counting || !out->right_counting ||
        !(out->left_csr & PWM_CH_CSR_EN) || !(out->right_csr & PWM_CH_CSR_EN))
        return E_IO;
    if (in_w(MOTOR_PWM_REG(left_slice, PWM_CHx_DIV)) != MOTOR_PWM_DIV_16 ||
        in_w(MOTOR_PWM_REG(right_slice, PWM_CHx_DIV)) != MOTOR_PWM_DIV_16 ||
        in_w(MOTOR_PWM_REG(left_slice, PWM_CHx_TOP)) != MOTOR_PWM_WRAP ||
        in_w(MOTOR_PWM_REG(right_slice, PWM_CHx_TOP)) != MOTOR_PWM_WRAP)
        return E_IO;
    for (UINT pin = MOTOR_LEFT_FORWARD_PIN; pin <= MOTOR_RIGHT_REVERSE_PIN; ++pin) {
        if (in_w(GPIO_CTRL(pin)) != GPIO_CTRL_FUNCSEL_PWM ||
            (in_w(GPIO(pin)) & GPIO_OD)) return E_IO;
    }
    return E_OK;
}
