/*
 *	subsystem5.h
 *	Subsystem 5 -- Ultrasonic scanning, obstacle profiling & avoidance.
 *	Student ID: 2500434.
 *
 *	Public interface: only what OTHER subsystems may use.
 *
 *	How S5 works with the Vehicle Controller (VC, in Buddy 2's
 *	robot_control_step()), interfaces IF-10 / IF-11 in TEAM.md:
 *	  - S5 never drives the motors.  It publishes ONE request word that
 *	    the VC reads every control step (subsystem5_get_request()).
 *	  - The VC answers through the S5-owned event flag flg_s5_evt.
 *	  - A request with a new seq is a new command; the VC acts on it once.
 *	  - S5 asks for HOLD from boot until its servo has finished its first
 *	    swing; the VC does not arm while S5 holds.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#define S5_PRI_MAIN 20 /* priority band 20-21 (TEAM.md) */

/* ---- S5 -> VC request (packed into one 32-bit word, single writer) ----
 *   bits 0-2   kind          S5_REQ_*        (5..7 reserved = invalid)
 *   bits 3-4   move          S5_MOVE_*
 *   bit  5     stop_on_line  MOVE only: stop as soon as an IR sensor
 *                            sees black, and report S5_EVT_LINE_FOUND
 *   bits 6-9   seq           changes on every new request
 *   bits 10-19 value         MOVE only: cm (FWD) or degrees (TURN)
 *   bits 20-27 unused, 0     (were the curved bypass's ARC fields)
 *   bit  28    line_all      with stop_on_line: stop only once EVERY
 *                            steering IR sensor has seen black during the
 *                            move (each may have cleared again), not the
 *                            first one.  For a crooked approach to the
 *                            line (2026-11-07).
 *   bits 29-31 unused, 0
 */
#define S5_REQ_NONE      (0U) /* no request: VC follows the line          */
#define S5_REQ_HOLD      (1U) /* stop and stay still (S5 is scanning)     */
#define S5_REQ_MOVE      (2U) /* do one move, then report                 */
#define S5_REQ_RESUME    (3U) /* avoidance over: back to line following   */
#define S5_REQ_SAFE_STOP (4U) /* no path / line lost: stop until reboot   */

#define S5_MOVE_FWD    (0U)
#define S5_MOVE_TURN_L (1U) /* pivot on one stopped wheel            */
#define S5_MOVE_TURN_R (2U)
                            /* move 3 reserved = invalid             */

#define S5_REQ_VALUE_MAX (1023U) /* 10-bit value field */
#define S5_REQ_SEQ_MASK  (0x0FU) /* 4-bit seq field    */

typedef struct
{
    UB   kind;           /* S5_REQ_*                        */
    UB   move;           /* S5_MOVE_*        (MOVE only)    */
    UB   seq;            /* 0..15                           */
    BOOL b_stop_on_line; /* (MOVE only)                     */
    UH   value;          /* cm or deg, 0..1023 (MOVE only)  */
    BOOL b_line_all;     /* (stop-on-line) every sensor     */
} S5_REQUEST;

/* ---- VC -> S5 event flag (owned and created by S5) ----
 * One producer per bit (the VC).  S5 clears the bits it waits for before
 * each new request, so an old reply can never answer a new one. */
#define S5_EVT_FOLLOWING   (1U << 0) /* VC armed and line-following now   */
#define S5_EVT_STOPPED     (1U << 1) /* HOLD: robot is at rest            */
#define S5_EVT_MOTION_DONE (1U << 2) /* MOVE finished                     */
#define S5_EVT_LINE_FOUND  (1U << 3) /* MOVE stopped early: line seen     */
#define S5_EVT_MOVE_FAILED (1U << 4) /* MOVE rejected or timed out        */

IMPORT ID flg_s5_evt;

/**
 * Read the current S5 request.  Any task; never blocks.
 * @return E_OK, or E_PAR if p_req is NULL or the word holds reserved
 *         values (the caller must then stop safely).
 */
IMPORT ER subsystem5_get_request(S5_REQUEST * p_req);

IMPORT void obstacle_task(INT stacd, void * exinf);

#endif /* SUBSYSTEM5_H */
