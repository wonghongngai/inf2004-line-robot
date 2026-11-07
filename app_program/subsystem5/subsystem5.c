/*
 *	subsystem5.c
 *	Subsystem 5 -- Ultrasonic scanning, obstacle profiling & avoidance.
 *
 *	subsystem5_init():  creates flg_s5_evt (VC -> S5 replies, IF-11).
 *	subsystem5_start(): starts obstacle_task, which owns the HC-SR04P and
 *	                    the SG90 and publishes the S5 request word (IF-10).
 *
 *	Record every pin, task, object and interface in TEAM.md.
 */

#include "subsystem5.h"

/* VC -> S5 replies (S5_EVT_* bits).  Owned by S5: the receiver owns. */
EXPORT ID flg_s5_evt;

/* Create subsystem 5's kernel objects */
EXPORT ER subsystem5_init(void)
{
    T_CFLG cflg = {0};

    cflg.flgatr  = TA_TFIFO | TA_WSGL; /* only obstacle_task waits */
    cflg.iflgptn = 0U;

    flg_s5_evt = tk_cre_flg(&cflg);
    return (flg_s5_evt < E_OK) ? flg_s5_evt : E_OK;
}

/* Start the obstacle task */
EXPORT ER subsystem5_start(void)
{
    ID tid = app_start_task(obstacle_task, S5_PRI_MAIN, 0, "obstacle");
    return (tid < E_OK) ? tid : E_OK;
}
