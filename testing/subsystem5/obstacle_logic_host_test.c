/*
 * obstacle_logic_host_test.c - S5-U45: subsystem 5's on-target checks on
 * the PC.
 *
 * Runs the real subsystem5_test() from app_program/subsystem5/subsystem5_test.c
 * against the real obstacle_logic.c and drv_servo.c, so the host and the Pico
 * run the same checks (filter, fine-scan window, profile, planner, bypass
 * table, servo maths, packed words).  Only the hardware calls below are fake.
 *
 * Build and run through the suite:  sh testing/run_host_tests.sh s5/
 * Exit 0 when every [TEST] line is PASS and at least one check ran.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "s5_host_stubs.h"

IMPORT void subsystem5_test(void);

static int g_checks;
static int g_failed;

/* The [TEST] lines are the result: count them as they are printed */
int tm_printf(UB *format, ...)
{
    char    line[512];
    va_list args;
    int     n;

    va_start(args, format);
    n = vsnprintf(line, sizeof(line), (const char *)format, args);
    va_end(args);

    if ((0 == strncmp(line, "[TEST] ", 7)) &&
        (NULL == strstr(line, " SUMMARY ")))
    {
        ++g_checks;
        if (NULL != strstr(line, " FAIL "))
        {
            ++g_failed;
        }
    }
    fputs(line, stdout);
    return n;
}

/* ---- Hardware boundary: no servo, no PWM, no timer on the PC ---------- */
ER tk_dly_tsk(RELTIM dlytim) { (void)dlytim; return E_OK; }
void out_w(UW addr, UW data) { (void)addr; (void)data; }
ER pwm_set_pin(unsigned int no) { (void)no; return E_OK; }
ER pwm_set_wrap(unsigned int no, UW wrap) { (void)no; (void)wrap; return E_OK; }
ER pwm_set_cc(unsigned int no, UW cc) { (void)no; (void)cc; return E_OK; }
ER pwm_set_enabled(unsigned int no, BOOL enable)
{ (void)no; (void)enable; return E_OK; }
void rp2040_timer_start(void) {}
UW rp2040_time_us(void) { return 0U; }
void rp2040_delay_us(UW us) { (void)us; }
void rp2040_pwm_unreset(void) {}

int main(void)
{
    subsystem5_test();
    printf("host: %d checks, %d failed\n", g_checks, g_failed);
    return ((g_checks > 0) && (0 == g_failed)) ? 0 : 1;
}
