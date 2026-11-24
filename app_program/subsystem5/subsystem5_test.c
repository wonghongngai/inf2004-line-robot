/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5.  Built only with: make TEST=1
 *  Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 */

#include "subsystem5.h"
#include "drv_servo.h"
#include "obstacle_logic.h"
#include "../test_report.h"

#if APP_TEST

#define SWEEP_DDEG     (1200U) /* a 120 deg glide (maths only)    */
#define PWM_PERIOD_MS  (20U)   /* the servo reads one pulse per 20 ms */
#define DEAD_BAND_DDEG (9U)    /* SG90 10 us dead band ~ 0.9 deg    */
#define MAX_MOVE_DDEG  (17U)   /* cruise 80 deg/s = 1.6 deg/period  */

/*
 * Sample a glide once per PWM period, as the servo sees it.  Reports
 * whether every period's change is above the dead band (no stick-and-
 * jump) and no faster than cruise, it never goes backwards, and it ends
 * exactly on the distance.  The final period may be a short remainder.
 */
static BOOL glide_is_smooth(UW distance_ddeg)
{
    UW   total_ms = servo_profile_total_ms(distance_ddeg);
    UW   last     = servo_profile_ddeg(distance_ddeg, 0U);
    UW   now;
    UW   t_ms;
    BOOL b_smooth = (0U == last) ? TRUE : FALSE;

    for (t_ms = PWM_PERIOD_MS; t_ms < total_ms; t_ms += PWM_PERIOD_MS)
    {
        now = servo_profile_ddeg(distance_ddeg, t_ms);
        if ((now < last) || ((now - last) < DEAD_BAND_DDEG) ||
            ((now - last) > MAX_MOVE_DDEG))
        {
            b_smooth = FALSE;
        }
        last = now;
    }

    return (b_smooth &&
            (distance_ddeg == servo_profile_ddeg(distance_ddeg, total_ms)))
               ? TRUE
               : FALSE;
}

/* Fill a coarse scan (50/70/90/110/130), all NO_ECHO */
static void make_coarse(scan_point_t * p_points)
{
    UW index;

    for (index = 0U; index < COARSE_POINTS; index++)
    {
        p_points[index].angle_deg   = SCAN_MIN_DEG + (index * COARSE_STEP_DEG);
        p_points[index].distance_mm = 0U;
        p_points[index].status      = ULTRASONIC_NO_ECHO;
    }
}

/* Make the coarse point at 'angle_deg' an OK reading of 'distance_mm' */
static void set_hit(scan_point_t * p_points, UW angle_deg, UW distance_mm)
{
    UW index = (angle_deg - SCAN_MIN_DEG) / COARSE_STEP_DEG;

    p_points[index].status      = ULTRASONIC_OK;
    p_points[index].distance_mm = distance_mm;
}

/* TRUE if fine_scan_window() gives exactly start..end for this scan */
static BOOL window_is(const scan_point_t * p_coarse, UW start_deg, UW end_deg)
{
    UW got_start = 0U;
    UW got_end   = 0U;

    return (fine_scan_window(p_coarse, COARSE_POINTS, &got_start, &got_end) &&
            (start_deg == got_start) && (end_deg == got_end))
               ? TRUE
               : FALSE;
}

/* Check that a request survives pack + unpack unchanged */
static BOOL request_round_trip(UB kind, UB move, UB seq, BOOL b_line, UH value)
{
    S5_REQUEST input = {kind, move, seq, b_line, value, FALSE, FALSE, FALSE};
    S5_REQUEST out   = {0U, 0U, 0U, FALSE, 0U, FALSE, FALSE, FALSE};

    if (!s5_request_unpack(s5_request_pack(&input), &out))
    {
        return FALSE;
    }
    return ((out.kind == kind) && (out.move == move) && (out.seq == seq) &&
            (out.b_stop_on_line == b_line) && (out.value == value))
               ? TRUE
               : FALSE;
}

/* 2026-11-06 bench fine scan (smoothed), 50..130 deg: box at 50..115,
 * a lone 501 mm echo at 120, far wall at 125/130 */
static const UW g_bench_1106_mm[] = {251U, 242U, 228U, 209U,  199U, 199U,
                                     190U, 190U, 203U, 203U,  203U, 199U,
                                     222U, 222U, 501U, 1279U, 1279U};
#define BENCH_1106_POINTS (sizeof(g_bench_1106_mm) / sizeof(g_bench_1106_mm[0]))

/* 2026-11-06 14:29 fine scan 50..130 deg of a 210 x 60 mm box (130: no
 * echo).  The robot then drove into it. */
static const UW g_live_1106_mm[] = {325U, 312U, 305U, 304U, 304U, 294U,
                                    294U, 294U, 297U, 302U, 302U, 302U,
                                    299U, 299U, 317U, 362U, 0U};
#define LIVE_1106_POINTS (sizeof(g_live_1106_mm) / sizeof(g_live_1106_mm[0]))

/* 2026-11-06 14:40 fine scan 50..110 deg (medians); the box runs past 50 */
static const UW g_cut_1106_mm[] = {302U, 302U, 299U, 299U, 294U, 299U, 299U,
                                   299U, 299U, 299U, 299U, 368U, 1019U};
#define CUT_1106_POINTS (sizeof(g_cut_1106_mm) / sizeof(g_cut_1106_mm[0]))

/* 2026-11-06 14:57 fine scan 50..130 deg (medians) of the 210 mm box */
static const UW g_turn_1106_mm[] = {359U, 359U,  302U,  299U,  291U, 289U,
                                    280U, 280U,  280U,  297U,  297U, 297U,
                                    600U, 2136U, 2136U, 1247U, 1238U};
#define TURN_1106_POINTS (sizeof(g_turn_1106_mm) / sizeof(g_turn_1106_mm[0]))

/* Fill a fine scan from 50 deg in 5 deg steps, all OK */
static void make_fine(scan_point_t * p_points, const UW * p_mm, UW count)
{
    UW index;

    for (index = 0U; index < count; index++)
    {
        p_points[index].angle_deg   = SCAN_MIN_DEG + (index * FINE_STEP_DEG);
        p_points[index].distance_mm = p_mm[index];
        p_points[index].status      = ULTRASONIC_OK;
    }
}

/* Run subsystem 5's self-tests */
EXPORT void subsystem5_test(void)
{
    scan_point_t coarse[COARSE_POINTS];
    UW           start_deg = 0U;
    UW           end_deg   = 0U;

    TEST_INIT();

    /* S5-U21: time-based glide profile */
    TEST_CHECK(
        "S5-U21", (1525U == servo_profile_total_ms(SWEEP_DDEG)),
        "120 deg glide takes 1525 ms (ramp 100 + cruise 1325 + ramp 100)");
    TEST_CHECK("S5-U21",
               ((0U == servo_profile_ddeg(SWEEP_DDEG, 0U)) &&
                (70U == servo_profile_ddeg(SWEEP_DDEG, 100U)) &&
                (600U == servo_profile_ddeg(SWEEP_DDEG, 762U)) &&
                (SWEEP_DDEG == servo_profile_ddeg(SWEEP_DDEG, 1525U)) &&
                (SWEEP_DDEG == servo_profile_ddeg(SWEEP_DDEG, 5000U))),
               "starts at 0, 7 deg after the ramp, half way at mid-time, then "
               "stays at the end");
    TEST_CHECK("S5-U21", glide_is_smooth(SWEEP_DDEG),
               "120 deg: every 20 ms pulse moves 0.9-1.7 deg, never back, "
               "lands exactly");
    TEST_CHECK("S5-U21", glide_is_smooth(600U), "60 deg glide is smooth too");
    TEST_CHECK("S5-U21",
               ((167U == servo_profile_total_ms(100U)) &&
                (50U == servo_profile_ddeg(100U, 83U)) &&
                glide_is_smooth(100U)),
               "short 10 deg move runs at the start speed throughout");
    TEST_CHECK("S5-U21",
               ((0U == servo_profile_total_ms(0U)) &&
                (0U == servo_profile_ddeg(0U, 50U))),
               "zero-length move takes no time");

    /* S5-U22: pulse width in tenths of a degree */
    TEST_CHECK("S5-U22", (1530U == servo_ddeg_to_pulse_us(900U)),
               "90.0 deg = 1530 us (forward)");
    TEST_CHECK("S5-U22",
               ((1077U == servo_ddeg_to_pulse_us(500U)) &&
                (1983U == servo_ddeg_to_pulse_us(1300U))),
               "50.0 / 130.0 deg = 1077 / 1983 us (the holder's limits)");
    TEST_CHECK("S5-U22", (1536U == servo_ddeg_to_pulse_us(905U)),
               "90.5 deg = 1536 us: sub-degree steps change the pulse");
    TEST_CHECK(
        "S5-U22",
        ((servo_ddeg_to_pulse_us(300U) == servo_ddeg_to_pulse_us(500U)) &&
         (servo_ddeg_to_pulse_us(1500U) == servo_ddeg_to_pulse_us(1300U))),
        "below 50 / above 130 deg is clamped (holder would hit the body)");
    TEST_CHECK(
        "S5-U22",
        ((servo_angle_to_pulse_us(20U) == servo_angle_to_pulse_us(50U)) &&
         (servo_angle_to_pulse_us(160U) == servo_angle_to_pulse_us(130U))),
        "whole-degree requests are clamped to 50..130 too");
    TEST_CHECK("S5-U22",
               (servo_angle_to_pulse_us(120U) == servo_ddeg_to_pulse_us(1200U)),
               "whole-degree and ddeg conversions agree");

    /* S5-U28: fine-scan angles from the coarse scan */
    make_coarse(coarse);
    TEST_CHECK("S5-U28",
               ((5U == COARSE_POINTS) && (50U == coarse[0].angle_deg) &&
                (70U == coarse[1].angle_deg) && (90U == coarse[2].angle_deg) &&
                (110U == coarse[3].angle_deg) && (130U == coarse[4].angle_deg)),
               "coarse scan is 50 / 70 / 90 / 110 / 130");
    make_coarse(coarse);
    set_hit(coarse, 90U, 350U);
    set_hit(coarse, 110U, 380U);
    TEST_CHECK("S5-U28", window_is(coarse, 70U, 130U),
               "hits at 90 and 110: fine 70..130 (one step past each)");
    make_coarse(coarse);
    set_hit(coarse, 50U, 212U);
    set_hit(coarse, 70U, 199U);
    set_hit(coarse, 90U, 184U);
    set_hit(coarse, 110U, 193U);
    TEST_CHECK("S5-U28", window_is(coarse, 50U, 130U),
               "2026-11-05 bench: hits 50..110, none at 130 -> fine 50..130");
    make_coarse(coarse);
    set_hit(coarse, 70U, 400U);
    set_hit(coarse, 110U, 350U);
    TEST_CHECK("S5-U28", window_is(coarse, 50U, 130U),
               "hits at 70 and 110 (90 clear): fine 50..130");
    make_coarse(coarse);
    set_hit(coarse, 90U, 300U);
    TEST_CHECK("S5-U28", window_is(coarse, 70U, 110U),
               "hit at 90 only: fine 70..110 (to both neighbours)");
    make_coarse(coarse);
    set_hit(coarse, 50U, 300U);
    TEST_CHECK("S5-U28", window_is(coarse, 50U, 70U),
               "hit at 50 only: fine 50..70 (limited at 50)");
    make_coarse(coarse);
    set_hit(coarse, 130U, 300U);
    TEST_CHECK("S5-U28", window_is(coarse, 110U, 130U),
               "hit at 130 only: fine 110..130 (limited at 130)");
    make_coarse(coarse);
    TEST_CHECK("S5-U28",
               !fine_scan_window(coarse, COARSE_POINTS, &start_deg, &end_deg),
               "no hits (all NO_ECHO): no fine scan");
    set_hit(coarse, 90U, 700U);
    TEST_CHECK("S5-U28",
               !fine_scan_window(coarse, COARSE_POINTS, &start_deg, &end_deg),
               "OK at 700 mm is beyond D_PROFILE_MM: not a hit");
    TEST_CHECK("S5-U28",
               (!fine_scan_window(NULL, COARSE_POINTS, &start_deg, &end_deg) &&
                !fine_scan_window(coarse, COARSE_POINTS, NULL, &end_deg) &&
                !fine_scan_window(coarse, COARSE_POINTS, &start_deg, NULL)),
               "NULL pointer gives FALSE");

    /* S5-U09: S5 -> VC request word pack/unpack (IF-10, CR1) */
    {
        S5_REQUEST req;

        TEST_CHECK(
            "S5-U09",
            request_round_trip(S5_REQ_MOVE, S5_MOVE_TURN_R, 9U, TRUE, 1023U),
            "MOVE turn-right 1023, seq 9, stop-on-line round trip");
        TEST_CHECK("S5-U09",
                   request_round_trip(S5_REQ_HOLD, S5_MOVE_FWD, 0U, FALSE, 0U),
                   "HOLD round trip");
        TEST_CHECK(
            "S5-U09",
            request_round_trip(S5_REQ_SAFE_STOP, S5_MOVE_FWD, 15U, FALSE, 0U),
            "SAFE_STOP, seq 15 round trip");
        TEST_CHECK("S5-U09", !s5_request_unpack(5U, &req),
                   "reserved kind 5 rejected");
        TEST_CHECK("S5-U09", !s5_request_unpack(7U, &req),
                   "reserved kind 7 rejected");
        TEST_CHECK("S5-U09",
                   !s5_request_unpack((UW)S5_REQ_MOVE | (3UL << 3), &req),
                   "reserved move 3 rejected");
        TEST_CHECK("S5-U09",
                   (!s5_request_unpack((UW)S5_REQ_MOVE | (1UL << 20), &req) &&
                    !s5_request_unpack((UW)S5_REQ_MOVE | (1UL << 27), &req)),
                   "unused bits 20-27 set: rejected");
        {
            S5_REQUEST all_in  = {S5_REQ_MOVE, S5_MOVE_FWD, 3U,    TRUE,
                                  50U,         TRUE,        FALSE, FALSE};
            S5_REQUEST all_out = {0U, 0U, 0U, FALSE, 0U, FALSE, FALSE, FALSE};

            TEST_CHECK("S5-U09",
                       (s5_request_unpack(s5_request_pack(&all_in), &all_out) &&
                        all_out.b_line_all && all_out.b_stop_on_line &&
                        (50U == all_out.value) &&
                        (0UL != (s5_request_pack(&all_in) & (1UL << 28)))),
                       "FWD 50 cm, stop when BOTH sensors crossed (bit 28) "
                       "round trip");
        }
        TEST_CHECK("S5-U09",
                   !s5_request_unpack((UW)S5_REQ_HOLD | (1UL << 29), &req),
                   "spin bit 29 on a HOLD: rejected");
        TEST_CHECK("S5-U09", (S5_REQ_NONE == req.kind),
                   "rejected word reads back as NONE");
        /* Spin (bit 29, 23 Nov): valid only on a MOVE that turns */
        {
            S5_REQUEST spin_in  = {S5_REQ_MOVE, S5_MOVE_TURN_R, 4U,   FALSE,
                                   90U,         FALSE,          TRUE, FALSE};
            S5_REQUEST spin_out = {0U, 0U, 0U, FALSE, 0U, FALSE, FALSE, FALSE};
            S5_REQUEST fwd_in   = {S5_REQ_MOVE, S5_MOVE_FWD, 4U,   FALSE,
                                   10U,         FALSE,       TRUE, FALSE};
            S5_REQUEST rev_in   = {S5_REQ_MOVE, S5_MOVE_FWD, 6U,    FALSE,
                                   15U,         FALSE,       FALSE, TRUE};
            S5_REQUEST turn_rev = {S5_REQ_MOVE, S5_MOVE_TURN_L, 6U,    FALSE,
                                   90U,         FALSE,          FALSE, TRUE};
            UW         word     = s5_request_pack(&spin_in);

            TEST_CHECK("S5-U09",
                       ((0UL != (word & (1UL << 29))) &&
                        s5_request_unpack(word, &spin_out) && spin_out.b_spin &&
                        (S5_MOVE_TURN_R == spin_out.move) &&
                        (90U == spin_out.value)),
                       "spin right 90: bit 29 set, round trip");
            spin_in.move = S5_MOVE_TURN_L;
            TEST_CHECK(
                "S5-U09",
                (s5_request_unpack(s5_request_pack(&spin_in), &spin_out) &&
                 spin_out.b_spin && (S5_MOVE_TURN_L == spin_out.move)),
                "spin left 90 round trip");
            TEST_CHECK("S5-U09",
                       (request_round_trip(S5_REQ_MOVE, S5_MOVE_TURN_L, 2U,
                                           FALSE, 90U) &&
                        s5_request_unpack(s5_request_pack(&(S5_REQUEST){
                                              S5_REQ_MOVE, S5_MOVE_TURN_L, 2U,
                                              FALSE, 90U, FALSE, FALSE, FALSE}),
                                          &spin_out) &&
                        !spin_out.b_spin),
                       "a turn without the bit is a pivot (b_spin FALSE)");
            TEST_CHECK(
                "S5-U09",
                (!s5_request_unpack(
                     (UW)S5_REQ_MOVE | (10UL << 10) | (1UL << 29), &req) &&
                 (0UL == (s5_request_pack(&fwd_in) & (1UL << 29)))),
                "spin on a FWD move: rejected, and pack never sets it");
            /* Reverse (bit 30, 23 Nov): valid only on a forward MOVE */
            word = s5_request_pack(&rev_in);
            TEST_CHECK("S5-U09",
                       ((0UL != (word & (1UL << 30))) &&
                        s5_request_unpack(word, &spin_out) &&
                        spin_out.b_reverse && !spin_out.b_spin &&
                        (S5_MOVE_FWD == spin_out.move) &&
                        (15U == spin_out.value)),
                       "reverse 15 cm: bit 30 set, round trip");
            TEST_CHECK(
                "S5-U09",
                (s5_request_unpack(s5_request_pack(&fwd_in), &spin_out) &&
                 !spin_out.b_reverse),
                "a leg without the bit goes forward (b_reverse FALSE)");
            TEST_CHECK(
                "S5-U09",
                (!s5_request_unpack((UW)S5_REQ_MOVE | (1UL << 3) | (1UL << 30),
                                    &req) &&
                 !s5_request_unpack((UW)S5_REQ_HOLD | (1UL << 30), &req) &&
                 (0UL == (s5_request_pack(&turn_rev) & (1UL << 30)))),
                "reverse on a turn or a HOLD: rejected, and pack never "
                "sets it there");
            TEST_CHECK("S5-U09",
                       !s5_request_unpack((UW)S5_REQ_MOVE | (1UL << 31), &req),
                       "unused bit 31 set: rejected");
        }
        req = (S5_REQUEST){S5_REQ_MOVE, S5_MOVE_FWD, 1U,    FALSE,
                           2000U,       FALSE,       FALSE, FALSE};
        TEST_CHECK("S5-U09",
                   (1023U == ((s5_request_pack(&req) >> 10) & 0x3FFU)),
                   "value over 1023 clamped");
        TEST_CHECK("S5-U09", ((UW)S5_REQ_SAFE_STOP == s5_request_pack(NULL)),
                   "pack(NULL) gives a SAFE_STOP word");
#ifndef S5_HOST_TEST
        /* Target only: subsystem5_get_request() lives in task_obstacle.c,
         * which the host test (S5-U45) does not build */
        TEST_CHECK("S5-U09", (E_PAR == subsystem5_get_request(NULL)),
                   "get_request(NULL) is E_PAR");
#endif
    }

    /* S5-U35: turn-and-box bypass move table */
    {
        static const UB left_moves[BYPASS_STEPS] = {
            S5_MOVE_TURN_L, S5_MOVE_FWD, S5_MOVE_TURN_R, S5_MOVE_FWD,
            S5_MOVE_TURN_R, S5_MOVE_FWD, S5_MOVE_FWD,    S5_MOVE_TURN_L};
        static const UB right_moves[BYPASS_STEPS] = {
            S5_MOVE_TURN_R, S5_MOVE_FWD, S5_MOVE_TURN_L, S5_MOVE_FWD,
            S5_MOVE_TURN_L, S5_MOVE_FWD, S5_MOVE_FWD,    S5_MOVE_TURN_R};
        static const UH values[BYPASS_STEPS] = {
            90U, 23U, 90U, PASS_MAX_CM, 90U, 50U, 4U, 120U}; /* F1: 170 + 60 */
        static const plan_t left_plan  = {PLAN_PASS_LEFT, 170U, FALSE};
        static const plan_t right_plan = {PLAN_PASS_RIGHT, 170U, FALSE};
        static const plan_t stop_plan  = {PLAN_STOP, 0U, FALSE};
        static const plan_t cont_plan  = {PLAN_CONTINUE, 0U, FALSE};
        bypass_step_t       step;
        UW                  index;
        BOOL                b_left_ok  = TRUE;
        BOOL                b_right_ok = TRUE;
        BOOL                b_line_ok  = TRUE;
        BOOL                b_all_ok   = TRUE;
        BOOL                b_flags_ok = TRUE;

        for (index = 0U; index < BYPASS_STEPS; index++)
        {
            if (!bypass_step(&left_plan, index, &step) ||
                (left_moves[index] != step.move) ||
                (values[index] != step.value))
            {
                b_left_ok = FALSE;
            }
            if (step.b_stop_on_line !=
                (((5U == index) || (7U == index)) ? TRUE : FALSE))
            {
                b_line_ok = FALSE;
            }
            if (step.b_line_all != ((5U == index) ? TRUE : FALSE))
            {
                b_all_ok = FALSE;
            }
            if ((step.b_watch_side != ((3U == index) ? TRUE : FALSE)) ||
                (step.b_check_ahead != ((4U == index) ? TRUE : FALSE)))
            {
                b_flags_ok = FALSE;
            }
            if (!bypass_step(&right_plan, index, &step) ||
                (right_moves[index] != step.move) ||
                (values[index] != step.value))
            {
                b_right_ok = FALSE;
            }
        }
        TEST_CHECK("S5-U35", b_left_ok,
                   "left pass, shift 170: L90, F29 (170 + 120), R90, F up to "
                   "80 looked, "
                   "R90, F50 to line, F4, L up to 120");
        TEST_CHECK("S5-U35", b_right_ok,
                   "right pass: the mirror (R90, F29, L90, F up to 80, L90, "
                   "F50, F4, R up to 120)");
        TEST_CHECK("S5-U35", b_line_ok,
                   "only F3 (step 6) and T4 (step 8) stop on the line");
        TEST_CHECK("S5-U35", b_all_ok,
                   "only F3 waits for BOTH sensors to cross the line");
        TEST_CHECK(
            "S5-U35", b_flags_ok,
            "F2 (step 4) is the looked pass; T3 (step 5) looks ahead after");
        TEST_CHECK("S5-U35",
                   (!bypass_step(&left_plan, BYPASS_STEPS, &step) &&
                    !bypass_step(&stop_plan, 0U, &step) &&
                    !bypass_step(&cont_plan, 0U, &step) &&
                    !bypass_step(NULL, 0U, &step) &&
                    !bypass_step(&left_plan, 0U, NULL)),
                   "index past the end, not a pass, or NULL: FALSE");
        TEST_CHECK("S5-U35",
                   ((17U == bypass_side_cm(170U)) &&
                    (18U == bypass_side_cm(171U)) &&
                    (BYPASS_SIDE_MIN_CM == bypass_side_cm(98U)) &&
                    (BYPASS_SIDE_MIN_CM == bypass_side_cm(0U)) &&
                    (BYPASS_SIDE_MAX_CM == bypass_side_cm(600U))),
                   "F1 = shift rounded up to cm, 10..40 cm (170 -> 17, 171 -> "
                   "18, 98 -> 10, 600 -> 40)");
        TEST_CHECK("S5-U35",
                   ((23U == bypass_out_cm(170U)) &&
                    (BYPASS_SIDE_MIN_CM == bypass_out_cm(0U)) &&
                    (34U == bypass_out_cm(280U)) &&
                    (BYPASS_SIDE_MAX_CM == bypass_out_cm(600U))),
                   "F1 as driven: shift + 60 (look spins clear the box; 120 "
                   "until 24 Nov), 10..40 cm (170 -> 23, 0 -> 10, 280 -> 34)");
    }

    /* S5-U31: profile + plan on the 2026-11-06 bench scan */
    {
        static scan_point_t fine[BENCH_1106_POINTS];
        obstacle_profile_t  prof;
        plan_t              plan;

        make_coarse(coarse); /* all inside the fine window: not used */
        make_fine(fine, g_bench_1106_mm, BENCH_1106_POINTS);
        (void)obstacle_profile(fine, BENCH_1106_POINTS, coarse, COARSE_POINTS,
                               &prof);
        obstacle_plan(&prof, MAX_RETRIES, &plan);
        TEST_CHECK("S5-U31",
                   (prof.b_valid && (190U == prof.closest_mm) &&
                    (80U == prof.closest_deg)),
                   "closest 190 mm at 80 deg");
        TEST_CHECK(
            "S5-U31", ((185U == prof.width_mm) && (-68 == prof.centre_y_mm)),
            "box is 50..115 deg: 501 mm echo not part of it (run reaches 50 "
            "deg: only the left edge pulled in, 185 mm wide, centre -68)");
        TEST_CHECK("S5-U31",
                   ((CLEARANCE_OPEN_MM == prof.clear_left_mm) &&
                    (CLEARANCE_UNSEEN_MM == prof.clear_right_mm)),
                   "lone 501 mm echo does not narrow the left; right never "
                   "seen (box runs to 50 deg)");
        TEST_CHECK("S5-U31",
                   ((PLAN_PASS_LEFT == plan.decision) &&
                    (149U == plan.shift_mm) && !plan.b_side_unseen),
                   "decision PASS LEFT (seen clear), shift 149 mm");

        /* Same scan, but a real object on the left seen at 120 AND 125 */
        fine[15].distance_mm = 505U;
        (void)obstacle_profile(fine, BENCH_1106_POINTS, coarse, COARSE_POINTS,
                               &prof);
        obstacle_plan(&prof, MAX_RETRIES, &plan);
        TEST_CHECK("S5-U31",
                   ((prof.clear_left_mm < PASS_GAP_MM) &&
                    (PLAN_PASS_RIGHT == plan.decision) && plan.b_side_unseen),
                   "confirmed object 501/505 mm on the left blocks it: PASS "
                   "RIGHT, flagged not seen");
    }

    /* S5-U32: the 21 cm x 6 cm box the robot drove into, 2026-11-06 14:29 */
    {
        static scan_point_t fine[LIVE_1106_POINTS];
        obstacle_profile_t  prof;
        plan_t              plan;

        make_coarse(coarse);
        make_fine(fine, g_live_1106_mm, LIVE_1106_POINTS);
        fine[LIVE_1106_POINTS - 1U].status = ULTRASONIC_NO_ECHO; /* 130 */
        (void)obstacle_profile(fine, LIVE_1106_POINTS, coarse, COARSE_POINTS,
                               &prof);
        obstacle_plan(&prof, MAX_RETRIES, &plan);
        TEST_CHECK("S5-U32",
                   (prof.b_valid && (294U == prof.closest_mm) &&
                    (75U == prof.closest_deg)),
                   "closest 294 mm at 75 deg");
        TEST_CHECK("S5-U32",
                   ((308U == prof.width_mm) && (-54 == prof.centre_y_mm)),
                   "run reaches 50 deg: right edge kept at -208, left 207 - "
                   "107 = 100 mm");
        TEST_CHECK(
            "S5-U32",
            ((PLAN_PASS_LEFT == plan.decision) && (225U == plan.shift_mm)),
            "PASS LEFT, shift 100 + 125 = 225 mm");
    }

    /* S5-U33: 2026-11-06 14:40, the box cut off by the 50 deg scan limit */
    {
        static scan_point_t fine[CUT_1106_POINTS];
        obstacle_profile_t  prof;
        plan_t              plan;

        make_coarse(coarse);
        coarse[3].distance_mm = 633U;  /* 110: inside the fine window */
        coarse[4].distance_mm = 2165U; /* 130: open on the left       */
        make_fine(fine, g_cut_1106_mm, CUT_1106_POINTS);
        (void)obstacle_profile(fine, CUT_1106_POINTS, coarse, COARSE_POINTS,
                               &prof);
        obstacle_plan(&prof, MAX_RETRIES, &plan);
        TEST_CHECK("S5-U33",
                   ((182U == prof.width_mm) && (-103 == prof.centre_y_mm)),
                   "right edge kept at -194 (not seen), left 95 - 107 = -12: "
                   "182 mm (was 75)");
        TEST_CHECK(
            "S5-U33",
            ((PLAN_PASS_LEFT == plan.decision) && (113U == plan.shift_mm)),
            "PASS LEFT, shift -12 + 125 = 113 mm");
    }

    /* S5-U34: 2026-11-06 14:57 scan of the 210 mm box */
    {
        static scan_point_t fine[TURN_1106_POINTS];
        obstacle_profile_t  prof;

        make_coarse(coarse);
        make_fine(fine, g_turn_1106_mm, TURN_1106_POINTS);
        (void)obstacle_profile(fine, TURN_1106_POINTS, coarse, COARSE_POINTS,
                               &prof);
        TEST_CHECK("S5-U34",
                   ((205U == prof.width_mm) && (-127 == prof.centre_y_mm) &&
                    (275 == prof.closest_x_mm)),
                   "width 205 mm (box 210), centre -127 mm, closest x 275 mm");
    }

    /* S5-U36 retired 23 Nov: the side watch's "passed the box" detector
     * went with the watched pass (F2 now spins to look, S5-U49) */

    /* S5-U37: F2 distances (side-watch checks removed 23 Nov) */
    {
        obstacle_profile_t prof  = {TRUE, 190U, 80U,   185,
                                    -88,  205U, 9999U, 9999U};
        plan_t             left  = {PLAN_PASS_LEFT, 135U, FALSE};
        plan_t             right = {PLAN_PASS_RIGHT, 135U, FALSE};
        plan_t             stop  = {PLAN_STOP, 0U, FALSE};

        TEST_CHECK(
            "S5-U37", (286U == bypass_side_gap_mm(&left, &prof)),
            "side gap, left pass: F1 200 (135 + 60, rounded up) + pivots 100 "
            "- left edge 14 = 286 mm");
        TEST_CHECK("S5-U37", (110U == bypass_side_gap_mm(&right, &prof)),
                   "right pass: F1 200 + 100 - right edge 190 = 110 mm");
        {
            obstacle_profile_t wide = {TRUE, 190U, 90U,   185,
                                       0,    900U, 9999U, 9999U};
            plan_t             near = {PLAN_PASS_LEFT, 0U, FALSE};

            TEST_CHECK("S5-U37",
                       ((50U == bypass_side_gap_mm(&near, &wide)) &&
                        (0U == bypass_side_gap_mm(&stop, &prof)) &&
                        (0U == bypass_side_gap_mm(NULL, &prof))),
                       "side gap at least 50 mm (900 mm box, F1 10 cm); not a "
                       "pass or NULL: 0");
        }
        TEST_CHECK("S5-U37", (35U == bypass_pass_plan_cm(&prof)),
                   "never seen: front 185 - 100 + 60 deep + 150 tail + 50 = "
                   "345 -> 35 cm");
        prof.closest_x_mm = 900;
        TEST_CHECK("S5-U37",
                   ((PASS_MAX_CM == bypass_pass_plan_cm(&prof)) &&
                    (10U == bypass_pass_plan_cm(NULL))),
                   "plan at most PASS_MAX_CM; NULL: 10 cm");
        TEST_CHECK("S5-U37", (426U == bypass_probe_mm(226U)),
                   "after T3, box still ahead if under 226 + 200 = 426 mm");
    }

    /* S5-U49: looked-pass distances and the spin that faces the box */
    {
        obstacle_profile_t prof = {TRUE, 190U, 80U,   185,
                                   -88,  205U, 9999U, 9999U};

        TEST_CHECK(
            "S5-U49", (20U == bypass_look_start_cm(&prof)),
            "first look: front 185 - 100 + sensor 80 + 30 = 195 -> 20 cm");
        prof.closest_x_mm = 900;
        TEST_CHECK("S5-U49", (PASS_MAX_CM == bypass_look_start_cm(&prof)),
                   "first look at most PASS_MAX_CM");
        prof.closest_x_mm = -50;
        TEST_CHECK("S5-U49",
                   ((0U == bypass_look_start_cm(&prof)) &&
                    (0U == bypass_look_start_cm(NULL))),
                   "front already behind the centre, or NULL: look at once");
        TEST_CHECK("S5-U49",
                   (bypass_look_ok(LOOK_MIN_GAP_MM) &&
                    !bypass_look_ok(LOOK_MIN_GAP_MM - 1U) &&
                    (170U == LOOK_MIN_GAP_MM)),
                   "room to spin from 120 sweep + 50 margin = 170 mm out");
        TEST_CHECK("S5-U49",
                   ((246U == bypass_look_limit_mm(226U)) &&
                    (100U == bypass_look_limit_mm(50U))),
                   "beside under (226 - 80) + 100 = 246 mm; gap under the "
                   "sensor offset: 100");
        TEST_CHECK(
            "S5-U49", (22U == bypass_after_look_cm()),
            "after the box: tail 150 - 80 + 50 + 100 sweep = 220 -> 22 cm");
        TEST_CHECK("S5-U49",
                   ((S5_MOVE_TURN_R == bypass_look_turn(PLAN_PASS_LEFT)) &&
                    (S5_MOVE_TURN_L == bypass_look_turn(PLAN_PASS_RIGHT)) &&
                    (S5_MOVE_FWD == bypass_look_turn(PLAN_STOP))),
                   "left pass spins right to face the box, right pass left; "
                   "not a pass: no look");
    }

    /* S5-U38: a side the scan never saw past is "not seen", not open */
    {
        static const UW     centred_mm[] = {210U, 205U, 202U, 200U, 200U, 200U,
                                            200U, 200U, 200U, 200U, 200U, 200U,
                                            200U, 200U, 202U, 205U, 210U};
        static scan_point_t fine[17];
        obstacle_profile_t  prof;
        plan_t              plan;

        make_coarse(coarse);
        make_fine(fine, centred_mm, 17U);
        (void)obstacle_profile(fine, 17U, coarse, COARSE_POINTS, &prof);
        obstacle_plan(&prof, MAX_RETRIES, &plan);
        TEST_CHECK("S5-U38",
                   ((CLEARANCE_UNSEEN_MM == prof.clear_left_mm) &&
                    (CLEARANCE_UNSEEN_MM == prof.clear_right_mm)),
                   "box runs to both 50 and 130 deg: both sides not seen");
        TEST_CHECK("S5-U38",
                   ((PLAN_PASS_LEFT == plan.decision) &&
                    (260U == plan.shift_mm) && plan.b_side_unseen),
                   "no side seen clear: still passes (equal shifts -> left), "
                   "flagged not seen");
    }

    /* S5-U11: whole-degree calibration maths and clamping */
    TEST_CHECK("S5-U11",
               ((1077U == servo_angle_to_pulse_us(50U)) &&
                (1303U == servo_angle_to_pulse_us(70U)) &&
                (1530U == servo_angle_to_pulse_us(90U)) &&
                (1757U == servo_angle_to_pulse_us(110U)) &&
                (1983U == servo_angle_to_pulse_us(130U))),
               "50/70/90/110/130 deg = 1077/1303/1530/1757/1983 us");
    TEST_CHECK("S5-U11",
               ((1077U == servo_angle_to_pulse_us(0U)) &&
                (1077U == servo_angle_to_pulse_us(49U)) &&
                (1983U == servo_angle_to_pulse_us(131U)) &&
                (1983U == servo_angle_to_pulse_us(180U))),
               "below 50 / above 130 deg clamped to the holder limits");

    /* S5-U12: settle time for a coarse jump */
    TEST_CHECK("S5-U12", (0U == servo_settle_ms(90U, 90U)), "no move: no wait");
    TEST_CHECK("S5-U12",
               ((70U == servo_settle_ms(60U, 30U)) &&
                (70U == servo_settle_ms(30U, 60U))),
               "30 deg either way = 50 + 20 margin = 70 ms (symmetric)");
    TEST_CHECK("S5-U12", (170U == servo_settle_ms(30U, 120U)),
               "90 deg = 150 + 20 = 170 ms");
    TEST_CHECK("S5-U12", (22U == servo_settle_ms(90U, 91U)),
               "1 deg = 1.7 ms rounds UP to 2, + 20 = 22 ms");

    /* S5-U13: 2-of-3 obstacle confirmation filter (NFR5-1, NFR5-5) */
    {
        UB   hist;
        BOOL b_got;
        UW   index;

        hist = 0U;
        TEST_CHECK("S5-U13",
                   !obstacle_filter_update(&hist, ULTRASONIC_OK, 150U),
                   "(a) one close reading does not confirm");
        hist = 0U;
        (void)obstacle_filter_update(&hist, ULTRASONIC_OK, 150U);
        TEST_CHECK("S5-U13", obstacle_filter_update(&hist, ULTRASONIC_OK, 150U),
                   "(b) two close readings in a row confirm");
        hist = 0U;
        (void)obstacle_filter_update(&hist, ULTRASONIC_OK, 150U);
        (void)obstacle_filter_update(&hist, ULTRASONIC_OK, 500U);
        TEST_CHECK("S5-U13", obstacle_filter_update(&hist, ULTRASONIC_OK, 150U),
                   "(c) close, far, close: two of the last three confirm");
        hist = 0U;
        (void)obstacle_filter_update(&hist, ULTRASONIC_OK, 150U);
        (void)obstacle_filter_update(&hist, ULTRASONIC_NO_ECHO, 0U);
        TEST_CHECK("S5-U13",
                   !obstacle_filter_update(&hist, ULTRASONIC_NO_ECHO, 0U),
                   "(d) close then two NO_ECHO: not confirmed");
        hist = 0U;
        (void)obstacle_filter_update(&hist, ULTRASONIC_OK, 150U);
        (void)obstacle_filter_update(&hist, ULTRASONIC_TOO_NEAR, 0U);
        b_got = obstacle_filter_update(&hist, ULTRASONIC_TOO_NEAR, 0U);
        TEST_CHECK("S5-U13", (!b_got && (1U == hist)),
                   "(e) TOO_NEAR is never close and leaves the history at 1");
        hist  = 0U;
        b_got = FALSE;
        for (index = 0U; index < 3U; index++)
        {
            b_got =
                b_got || obstacle_filter_update(&hist, ULTRASONIC_NO_ECHO, 0U);
        }
        for (index = 0U; index < 3U; index++)
        {
            b_got =
                b_got || obstacle_filter_update(&hist, ULTRASONIC_TOO_FAR, 0U);
        }
        TEST_CHECK("S5-U13", !b_got,
                   "(f) NO_ECHO x3 and TOO_FAR x3 never confirm");
        hist = 0U;
        (void)obstacle_filter_update(&hist, ULTRASONIC_OK, D_DETECT_MM);
        TEST_CHECK("S5-U13",
                   !obstacle_filter_update(&hist, ULTRASONIC_OK, D_DETECT_MM),
                   "(g) 200 mm x2: not under D_DETECT_MM (boundary)");
        TEST_CHECK("S5-U13", !obstacle_filter_update(NULL, ULTRASONIC_OK, 150U),
                   "(h) NULL history is FALSE");
        hist = 0U;
        (void)obstacle_filter_update(&hist, ULTRASONIC_OK, 150U);
        (void)obstacle_filter_update(&hist, ULTRASONIC_BUSY, 0U);
        TEST_CHECK("S5-U13", obstacle_filter_update(&hist, ULTRASONIC_OK, 150U),
                   "(i) BUSY between two close readings is skipped");
    }

    /* S5-U08: planner decisions from fixed profiles.  Corridor half
     * = 150 / 2 + 50 = 125 mm; a side fits from PASS_GAP_MM = 250 mm. */
    {
        obstacle_profile_t prof;
        plan_t             plan;

        /* 100 mm box centred, both sides open */
        prof = (obstacle_profile_t){TRUE,
                                    250U,
                                    90U,
                                    250,
                                    0,
                                    100U,
                                    CLEARANCE_OPEN_MM,
                                    CLEARANCE_OPEN_MM};
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK("S5-U08",
                   ((PLAN_PASS_LEFT == plan.decision) &&
                    (175U == plan.shift_mm) && !plan.b_side_unseen),
                   "(a) centred, both open: equal shifts 175 -> LEFT");

        prof.centre_y_mm = -40; /* box a little to the right */
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK(
            "S5-U08",
            ((PLAN_PASS_LEFT == plan.decision) && (135U == plan.shift_mm)),
            "(b) box 40 mm right: LEFT, shift 10 + 125 = 135");

        prof.centre_y_mm = 40; /* box a little to the left */
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK(
            "S5-U08",
            ((PLAN_PASS_RIGHT == plan.decision) && (135U == plan.shift_mm)),
            "(c) box 40 mm left: RIGHT, shift 135");

        prof.centre_y_mm   = -40;
        prof.clear_left_mm = PASS_GAP_MM - 1U; /* left too narrow */
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK("S5-U08",
                   ((PLAN_PASS_RIGHT == plan.decision) &&
                    (215U == plan.shift_mm) && !plan.b_side_unseen),
                   "(d) left gap 249 < 250: RIGHT even though longer (215)");

        prof.clear_left_mm  = PASS_GAP_MM; /* exactly fits */
        prof.clear_right_mm = 100U;
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK(
            "S5-U08",
            ((PLAN_PASS_LEFT == plan.decision) && (135U == plan.shift_mm)),
            "(e) left gap exactly 250 fits: LEFT");

        prof.clear_left_mm  = 100U; /* both blocked */
        prof.clear_right_mm = 100U;
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK("S5-U08", (PLAN_REVERSE == plan.decision),
                   "(f) both blocked, retries left: REVERSE");
        obstacle_plan(&prof, MAX_RETRIES, &plan);
        TEST_CHECK("S5-U08",
                   ((PLAN_STOP == plan.decision) && (0U == plan.shift_mm)),
                   "(g) both blocked, retries used up: STOP (NO PATH)");

        prof.clear_left_mm  = CLEARANCE_OPEN_MM;
        prof.clear_right_mm = CLEARANCE_OPEN_MM;
        prof.centre_y_mm    = 300; /* right edge 250 > 125 */
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK("S5-U08", (PLAN_CONTINUE == plan.decision),
                   "(h) box wholly left of the corridor: CONTINUE (A5-1)");
        prof.centre_y_mm = -300;
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK("S5-U08", (PLAN_CONTINUE == plan.decision),
                   "(i) wholly right of the corridor: CONTINUE");
        prof.centre_y_mm = 175; /* right edge exactly 125: still in */
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK("S5-U08", (PLAN_CONTINUE != plan.decision),
                   "(j) edge exactly on the corridor line counts as in");

        prof.b_valid = FALSE;
        obstacle_plan(&prof, 0U, &plan);
        TEST_CHECK("S5-U08", (PLAN_CONTINUE == plan.decision),
                   "(k) nothing found (invalid profile): CONTINUE");
        obstacle_plan(NULL, 0U, &plan);
        TEST_CHECK("S5-U08", (PLAN_STOP == plan.decision),
                   "(l) NULL profile: STOP; NULL plan does not crash");
        obstacle_plan(&prof, 0U, NULL);
    }

    /* S5-U03: echo time -> status and distance (NFR5-5).  d = t x 343 / 2000 */
    {
        UW dist_mm = 7U;

        TEST_CHECK(
            "S5-U03",
            ((ULTRASONIC_NO_ECHO == ultrasonic_classify(0U, FALSE, &dist_mm)) &&
             (0U == dist_mm)),
            "(a) 0 us (ECHO never high): NO_ECHO, 0 mm");
        dist_mm = 7U;
        TEST_CHECK("S5-U03",
                   ((ULTRASONIC_NO_ECHO ==
                     ultrasonic_classify(1750U, TRUE, &dist_mm)) &&
                    (0U == dist_mm)),
                   "(b) ECHO still high at the window end: NO_ECHO");
        TEST_CHECK("S5-U03",
                   ((ULTRASONIC_TOO_NEAR ==
                     ultrasonic_classify(116U, FALSE, &dist_mm)) &&
                    (0U == dist_mm)),
                   "(c) 116 us = 19 mm < 2 cm: TOO_NEAR, 0 mm");
        TEST_CHECK(
            "S5-U03",
            ((ULTRASONIC_OK == ultrasonic_classify(117U, FALSE, &dist_mm)) &&
             (20U == dist_mm)),
            "(d) 117 us = 20 mm: OK (lower limit is valid)");
        TEST_CHECK(
            "S5-U03",
            ((ULTRASONIC_OK == ultrasonic_classify(1750U, FALSE, &dist_mm)) &&
             (300U == dist_mm)),
            "(e) 1750 us = 300 mm: OK");
        TEST_CHECK(
            "S5-U03",
            ((ULTRASONIC_OK == ultrasonic_classify(23324U, FALSE, &dist_mm)) &&
             (4000U == dist_mm)),
            "(f) 23324 us = 4000 mm: OK (upper limit is valid)");
        TEST_CHECK("S5-U03",
                   ((ULTRASONIC_TOO_FAR ==
                     ultrasonic_classify(23330U, FALSE, &dist_mm)) &&
                    (0U == dist_mm) &&
                    (ULTRASONIC_TOO_FAR ==
                     ultrasonic_classify(65535U, FALSE, &dist_mm))),
                   "(g) 4001 mm and the 65535 us counter limit: TOO_FAR, 0 mm");
        TEST_CHECK(
            "S5-U03",
            (ULTRASONIC_NO_ECHO == ultrasonic_classify(1750U, FALSE, NULL)),
            "(h) NULL distance pointer: NO_ECHO");
    }

    /* S5-U46: start-up self-test decision (#47) */
    {
        static const UW none[SELFTEST_PINGS]  = {0U, 0U, 0U, 0U, 0U};
        static const UW one[SELFTEST_PINGS]   = {0U, 0U, 1748U, 0U, 0U};
        static const UW two[SELFTEST_PINGS]   = {0U, 1748U, 0U, 30000U, 0U};
        static const UW all[SELFTEST_PINGS]   = {1748U, 1750U, 1747U, 1749U,
                                                 1748U};
        static const UW open_[SELFTEST_PINGS] = {30000U, 0U, 30000U, 0U,
                                                 30000U};

        TEST_CHECK("S5-U46", !s5_selftest_echo_ok(none, SELFTEST_PINGS),
                   "ECHO never high (unplugged / dead): FAIL");
        TEST_CHECK("S5-U46", !s5_selftest_echo_ok(one, SELFTEST_PINGS),
                   "one echo in 5 (a glitch): FAIL");
        TEST_CHECK("S5-U46", s5_selftest_echo_ok(two, SELFTEST_PINGS),
                   "two echoes in 5: PASS (SELFTEST_MIN_ECHOES = 2)");
        TEST_CHECK("S5-U46", s5_selftest_echo_ok(all, SELFTEST_PINGS),
                   "every ping answered (box at 30 cm): PASS");
        TEST_CHECK("S5-U46", s5_selftest_echo_ok(open_, SELFTEST_PINGS),
                   "open space: ECHO high to the window end, every other "
                   "ping BUSY: PASS");
        TEST_CHECK("S5-U46",
                   (!s5_selftest_echo_ok(NULL, SELFTEST_PINGS) &&
                    !s5_selftest_echo_ok(all, 0U) &&
                    !s5_selftest_echo_ok(all, 1U)),
                   "NULL, 0 pings or 1 ping: FAIL");
    }

    /* S5-U39: obstacle telemetry word (#40) */
    {
        obstacle_profile_t profile = {0};
        S5_OBSTACLE_INFO   info    = {7U, 7U, 7U};
        S5_OBSTACLE_INFO   out     = {0U, 0U, 0U};

        TEST_CHECK("S5-U39",
                   ((!s5_obstacle_from_profile(&profile, &info)) &&
                    (7U == info.distance_cm) && (7U == info.angle_deg) &&
                    (7U == info.width_cm)),
                   "invalid profile (nothing found) leaves the result as it "
                   "was");

        profile.b_valid     = TRUE;
        profile.closest_mm  = 294U;
        profile.closest_deg = 85U;
        profile.width_mm    = 215U;
        TEST_CHECK("S5-U39",
                   (s5_obstacle_from_profile(&profile, &info) &&
                    (29U == info.distance_cm) && (85U == info.angle_deg) &&
                    (22U == info.width_cm)),
                   "294 mm -> 29 cm, 215 mm -> 22 cm (rounded), angle kept");

        profile.closest_mm  = 9999U;
        profile.closest_deg = 200U;
        profile.width_mm    = 4005U;
        TEST_CHECK("S5-U39",
                   (s5_obstacle_from_profile(&profile, &info) &&
                    (S5_OBS_DISTANCE_MAX_CM == info.distance_cm) &&
                    (S5_OBS_ANGLE_MAX_DEG == info.angle_deg) &&
                    (S5_OBS_WIDTH_MAX_CM == info.width_cm)),
                   "over-range values clamp to 400 cm / 180 deg / 400 cm");

        info.distance_cm = 29U;
        info.angle_deg   = 85U;
        info.width_cm    = 22U;
        TEST_CHECK("S5-U39",
                   (s5_obstacle_unpack(s5_obstacle_pack(&info), &out) &&
                    (29U == out.distance_cm) && (85U == out.angle_deg) &&
                    (22U == out.width_cm)),
                   "pack + unpack round trip");

        info.distance_cm = 400U;
        info.angle_deg   = 180U;
        info.width_cm    = 400U;
        TEST_CHECK("S5-U39",
                   (s5_obstacle_unpack(s5_obstacle_pack(&info), &out) &&
                    (400U == out.distance_cm) && (180U == out.angle_deg) &&
                    (400U == out.width_cm)),
                   "maximum values round trip");

        info.distance_cm = 999U;
        TEST_CHECK("S5-U39",
                   (s5_obstacle_unpack(s5_obstacle_pack(&info), &out) &&
                    (400U == out.distance_cm)),
                   "pack clamps an out-of-range distance");

        TEST_CHECK("S5-U39",
                   ((0U == s5_obstacle_pack(NULL)) &&
                    s5_obstacle_unpack(0U, &out) && (0U == out.distance_cm) &&
                    (0U == out.angle_deg) && (0U == out.width_cm)),
                   "NULL packs to 0; 0 unpacks to 'no obstacle yet'");

        TEST_CHECK("S5-U39",
                   ((!s5_obstacle_unpack(1UL << 26U, &out)) &&
                    (0U == out.distance_cm) &&
                    (!s5_obstacle_unpack(401UL, &out)) &&
                    (!s5_obstacle_unpack(181UL << 9U, &out)) &&
                    (!s5_obstacle_unpack(401UL << 17U, &out)) &&
                    (!s5_obstacle_unpack(0U, NULL))),
                   "rejects unused bits, out-of-range fields and NULL");
    }

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
