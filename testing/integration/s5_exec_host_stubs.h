/*
 * s5_exec_host_stubs.h - INT-13 (int/robot_avoidance): the kernel boundary
 * of the real S5 executor (app_program/subsystem5/s5_executor.c).
 *
 * Force-included after robot_host_stubs.h.  The executor talks to S5 only
 * through g_flg_s5_evt (tk_set_flg / tk_clr_flg) and subsystem5_get_request();
 * robot_host_test.c (built with S5_REAL_EXECUTOR=1) fakes exactly those
 * three, so the controller and the executor run as production C.
 */
#ifndef S5_EXEC_HOST_STUBS_H
#define S5_EXEC_HOST_STUBS_H

#define S5_REAL_EXECUTOR 1

/* Block the real kernel header (obstacle_logic.h includes it):
 * robot_host_stubs.h supplies the types. */
#define __TK_TKERNEL_H__

typedef unsigned int UINT;

ER tk_set_flg(ID flgid, UINT setptn);
ER tk_clr_flg(ID flgid, UINT clrptn);

#endif /* S5_EXEC_HOST_STUBS_H */
