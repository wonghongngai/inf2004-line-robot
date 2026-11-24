/* Buddy 1: Robo Pico piezo buzzer on GP22 (PWM slice 3, channel A).
 *
 * The piezo is passive, so it needs a square wave: 2 kHz at 50% duty. The
 * tone stops by setting the compare value to 0 from a one-shot alarm handler,
 * so callers (the Wi-Fi task) never wait for the beep to finish.
 *
 * Slice 3 is otherwise unused: the motors use slices 4-5 (GP8-GP11), the
 * servo slice 7 (GP14) and the echo timer slice 0 (GP17). GP6 shares slice 3
 * but is an SIO encoder input, so the PWM output only reaches GP22. */
#include "drv_buzzer.h"

#define BUZZER_PIN        22U /* Robo Pico piezo (TEAM.md pin table) */
#define BUZZER_SLICE      ((BUZZER_PIN >> 1U) & 0x7U)
#define BUZZER_SLICE_BASE (PWM_BASE + (0x14UL * BUZZER_SLICE))
#define BUZZER_CSR_ADDR   (BUZZER_SLICE_BASE + PWM_CHx_CSR)
#define BUZZER_DIV_ADDR   (BUZZER_SLICE_BASE + PWM_CHx_DIV)
#define PWM_DIV_INT_SHIFT 4U     /* DIV integer part is bits 11:4 */
#define BUZZER_CLKDIV_INT 125UL  /* 125 MHz / 125 = 1 count per us */
#define BUZZER_TONE_HZ    2000UL /* near a typical piezo's loudest point */
#define BUZZER_TOP        (1000000UL / BUZZER_TONE_HZ - 1UL)
#define BUZZER_ON_CC      ((BUZZER_TOP + 1UL) / 2UL) /* 50% duty */
#define WIFI_BEEP_MS      120U
#if (BUZZER_PIN & 1U) != 0U
#error "BUZZER_PIN must be even (PWM channel A)"
#endif

LOCAL ID g_buzzer_alarm;

/* Alarm handler: end the tone. Only a register write, safe in handler context. */
LOCAL void buzzer_off(void * p_exinf)
{
    (void)p_exinf;
    (void)pwm_set_cc(BUZZER_PIN, 0U);
}

/* First use sets up the PWM slice (silent) and the stop alarm. */
LOCAL ER buzzer_init(void)
{
    T_CALM alarm = {0};
    ER     ercd;
    ID     alarm_id;
    if (g_buzzer_alarm > 0)
    {
        return E_OK;
    }
    ercd = pwm_set_pin(BUZZER_PIN);
    if (ercd < E_OK)
    {
        return ercd;
    }
    out_w(BUZZER_CSR_ADDR, 0U); /* free-running, disabled while configuring */
    out_w(BUZZER_DIV_ADDR, BUZZER_CLKDIV_INT << PWM_DIV_INT_SHIFT);
    (void)pwm_set_wrap(BUZZER_PIN, BUZZER_TOP);
    (void)pwm_set_cc(BUZZER_PIN, 0U);
    (void)pwm_set_enabled(BUZZER_PIN, TRUE);
    alarm.almatr = TA_HLNG;
    alarm.almhdr = (FP)buzzer_off;
    alarm_id     = tk_cre_alm(&alarm);
    if (alarm_id < E_OK)
    {
        return alarm_id;
    }
    g_buzzer_alarm = alarm_id;
    return E_OK;
}

EXPORT ER buzzer_beep(UINT duration_ms)
{
    ER ercd;
    if (duration_ms == 0U)
    {
        return E_PAR;
    }
    ercd = buzzer_init();
    if (ercd < E_OK)
    {
        return ercd;
    }
    (void)pwm_set_cc(BUZZER_PIN, BUZZER_ON_CC);
    ercd = tk_sta_alm(g_buzzer_alarm, (RELTIM)duration_ms);
    if (ercd < E_OK)
    {
        buzzer_off(NULL); /* never leave the tone running */
    }
    return ercd;
}

/* Overrides the weak default in cyw43_utk.c. Runs on the Wi-Fi task. */
EXPORT void wifi_utk_link_up(void)
{
#if WIFI_CONNECT_BEEP
    ER ercd = buzzer_beep(WIFI_BEEP_MS);
    if (ercd < E_OK)
    {
        LOG_ERR(TAG_S1_BUZZER, "connect beep failed, er=%d", (int)ercd);
    }
#endif
}
