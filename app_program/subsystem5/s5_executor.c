/**
 * @file    s5_executor.c
 * @brief   Shared executor for S5's requests.  See s5_executor.h.
 */

#include "s5_executor.h"

/* Tell S5 whether the robot is line-following */
EXPORT void s5_exec_set_following(S5_EXEC * p_x, BOOL b_following)
{
    if ((NULL == p_x) || (b_following == p_x->b_following))
    {
        return; /* flag calls only on change */
    }
    p_x->b_following = b_following;
    if (b_following)
    {
        (void)tk_set_flg(flg_s5_evt, S5_EVT_FOLLOWING);
    }
    else
    {
        (void)tk_clr_flg(flg_s5_evt, ~S5_EVT_FOLLOWING);
    }
}

/* TRUE while S5 holds at start-up */
EXPORT BOOL s5_exec_startup_hold(void)
{
    S5_REQUEST req;

    return ((E_OK != subsystem5_get_request(&req)) ||
            (S5_REQ_HOLD == req.kind))
               ? TRUE
               : FALSE;
}

/* Stop and reply: the current request is finished */
static void exec_finish(S5_EXEC * p_x, const S5_EXEC_OPS * p_ops,
                        void * p_ctx, UINT bits)
{
    p_ops->stop(p_ctx);
    p_x->b_replied = TRUE;
    (void)tk_set_flg(flg_s5_evt, bits);
}

/* One control step */
EXPORT BOOL s5_exec_step(S5_EXEC * p_x, const S5_EXEC_OPS * p_ops,
                         void * p_ctx, UW now_ms)
{
    S5_REQUEST req;
    BOOL       b_fresh;
    UB         result;

    if ((NULL == p_x) || (NULL == p_ops))
    {
        return FALSE;
    }

    /* Bad word or SAFE_STOP: latched stop */
    if ((E_OK != subsystem5_get_request(&req)) ||
        (S5_REQ_SAFE_STOP == req.kind))
    {
        s5_exec_set_following(p_x, FALSE);
        p_ops->safe_stop(p_ctx,
                         "obstacle: no path, line lost or bad S5 request");
        return TRUE;
    }

    /* No request, or avoidance over */
    if ((S5_REQ_NONE == req.kind) || (S5_REQ_RESUME == req.kind))
    {
        if (p_x->b_active)
        {
            p_x->b_active = FALSE;
            p_ops->stop(p_ctx);
            (void)p_ops->give_back(p_ctx); /* failure stops by itself */
            return TRUE;
        }
        s5_exec_set_following(p_x, p_ops->may_follow(p_ctx));
        return FALSE;
    }

    /* HOLD or MOVE: S5 has control */
    b_fresh = ((!p_x->b_active) || (req.seq != p_x->seq)) ? TRUE : FALSE;
    if (!p_x->b_active)
    {
        p_x->b_active = TRUE;
        s5_exec_set_following(p_x, FALSE);
        if (!p_ops->take_control(p_ctx))
        {
            return TRUE; /* the controller has stopped safely */
        }
    }

    if (b_fresh)
    {
        /* A new request cuts any running move short */
        p_ops->stop(p_ctx);
        p_x->seq        = req.seq;
        p_x->b_replied  = FALSE;
        p_x->started_ms = now_ms;
        p_x->line_seen  = 0U;

        if (S5_REQ_MOVE == req.kind)
        {
            if (!p_ops->start_move(p_ctx, &req))
            {
                exec_finish(p_x, p_ops, p_ctx, S5_EVT_MOVE_FAILED);
                return TRUE;
            }
        }
        return TRUE;
    }

    if (p_x->b_replied)
    {
        return TRUE; /* answered: wait for S5's next request */
    }

    if (S5_REQ_HOLD == req.kind)
    {
        if (p_ops->is_stopped(p_ctx) &&
            ((UW)(now_ms - p_x->started_ms) >= p_ops->hold_settle_ms))
        {
            exec_finish(p_x, p_ops, p_ctx, S5_EVT_STOPPED);
        }
        return TRUE;
    }

    /* MOVE in progress */
    if ((UW)(now_ms - p_x->started_ms) > p_ops->move_timeout_ms)
    {
        exec_finish(p_x, p_ops, p_ctx, S5_EVT_MOVE_FAILED);
        return TRUE;
    }
    if (req.b_stop_on_line)
    {
        UB now_on = p_ops->line_sensors(p_ctx);

        /* line_all: each steering sensor must have seen black at some
         * point in this move (they cross one after another when the robot
         * meets the line at an angle).  Otherwise the first one will do. */
        p_x->line_seen |= now_on;
        if ((req.b_line_all && (0U != p_ops->line_all_mask))
                ? ((p_x->line_seen & p_ops->line_all_mask) ==
                   p_ops->line_all_mask)
                : (0U != now_on))
        {
            exec_finish(p_x, p_ops, p_ctx, S5_EVT_LINE_FOUND);
            return TRUE;
        }
    }

    result = p_ops->run_move(p_ctx, &req);
    if (S5_EXEC_DONE == result)
    {
        exec_finish(p_x, p_ops, p_ctx, S5_EVT_MOTION_DONE);
    }
    else if (S5_EXEC_FAILED == result)
    {
        exec_finish(p_x, p_ops, p_ctx, S5_EVT_MOVE_FAILED);
    }
    else
    {
        /* still running */
    }
    return TRUE;
}
