/**
 * @file    s5_executor.h
 * @brief   Shared executor for S5's requests (IF-10 / IF-11).
 *
 * Whoever owns the motors (the VC in robot_control.c, or the all_test
 * loop in all_buddies_test.c) calls s5_exec_step() once per control step.
 * This module holds the whole hand-shake: HOLD / MOVE / RESUME /
 * SAFE_STOP, sequence numbers, move timeout, stop-on-line, the FOLLOWING
 * bit and the reply bits.  It never touches hardware:
 * the controller passes its own drive functions in an S5_EXEC_OPS table,
 * so each build keeps its own way of driving.
 *
 * Only the motor-owning task may call these functions.
 */

#ifndef S5_EXECUTOR_H
#define S5_EXECUTOR_H

#include "subsystem5.h"


/* What run_move() reports */
#define S5_EXEC_RUNNING (0U)
#define S5_EXEC_DONE    (1U)
#define S5_EXEC_FAILED  (2U)

/* The controller's drive functions.  'p_ctx' is passed back unchanged. */
typedef struct
{
    /* Motors off now (may be called every step). */
    void (*stop)(void * p_ctx);
    /* Begin a move (p_req->move): forward 'value' cm or pivot 'value'
     * deg.  Called once, with the robot stopped.  FALSE = rejected. */
    BOOL (*start_move)(void * p_ctx, const S5_REQUEST * p_req);
    /* Advance the move by one step: S5_EXEC_RUNNING / _DONE / _FAILED. */
    UB (*run_move)(void * p_ctx, const S5_REQUEST * p_req);
    /* TRUE once the robot is at rest after stop(). */
    BOOL (*is_stopped)(void * p_ctx);
    /* Steering IR sensors on black now, one bit each (stop-on-line). */
    UB (*line_sensors)(void * p_ctx);
    /* S5 takes over: pause line following.  FALSE = failed (the
     * controller has already stopped safely). */
    BOOL (*take_control)(void * p_ctx);
    /* S5 is done: resume line following.  FALSE = failed (as above). */
    BOOL (*give_back)(void * p_ctx);
    /* Latched safety stop with a reason. */
    void (*safe_stop)(void * p_ctx, const char * p_reason);
    /* TRUE while the robot is following the line normally (not turning,
     * not waiting for the line): S5 may look for obstacles then. */
    BOOL (*may_follow)(void * p_ctx);
    /* HOLD replies "stopped" no sooner than this after stop(). */
    UW hold_settle_ms;
    /* A MOVE running longer than this fails (S5_EVT_MOVE_FAILED).  Set to
     * suit the controller's speed (all_test drives at low power); S5
     * itself waits up to 25 s, so keep it below that. */
    UW move_timeout_ms;
    /* Every steering sensor's bit in line_sensors(): what a line_all
     * request waits for. */
    UB line_all_mask;
} S5_EXEC_OPS;

/* Executor state, one per controller (zero-initialise it) */
typedef struct
{
    BOOL b_active;    /* S5 has control                          */
    BOOL b_replied;   /* current request already answered        */
    BOOL b_following; /* last FOLLOWING value sent to S5         */
    UB   seq;         /* seq of the request being executed       */
    UW   started_ms;  /* when it started                         */
    UB   line_seen;   /* sensors that have seen black this move  */
} S5_EXEC;

/**
 * One control step.  Call after the controller's own safety checks, before
 * its line following.
 * @return TRUE while S5 has control: skip line following this step.
 */
IMPORT BOOL s5_exec_step(S5_EXEC * p_x, const S5_EXEC_OPS * p_ops,
                         void * p_ctx, UW now_ms);

/** Tell S5 whether the robot is line-following (call FALSE on any stop).
 *  Flag calls only on change. */
IMPORT void s5_exec_set_following(S5_EXEC * p_x, BOOL b_following);

/** TRUE while S5 holds at start-up (servo swing): don't arm yet. */
IMPORT BOOL s5_exec_startup_hold(void);

#endif /* S5_EXECUTOR_H */
