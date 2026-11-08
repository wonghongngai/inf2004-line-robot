/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
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
    S5_REQUEST in  = {kind, move, seq, b_line, value};
    S5_REQUEST out = {0U, 0U, 0U, FALSE, 0U};

    if (!s5_request_unpack(s5_request_pack(&in), &out))
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
            S5_REQUEST all_in = {S5_REQ_MOVE, S5_MOVE_FWD, 3U, TRUE, 50U, TRUE};
            S5_REQUEST all_out = {0U, 0U, 0U, FALSE, 0U, FALSE};

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
                   "unused bits 29-31 set: rejected");
        TEST_CHECK("S5-U09", (S5_REQ_NONE == req.kind),
                   "rejected word reads back as NONE");
        req = (S5_REQUEST){S5_REQ_MOVE, S5_MOVE_FWD, 1U, FALSE, 2000U};
        TEST_CHECK("S5-U09",
                   (1023U == ((s5_request_pack(&req) >> 10) & 0x3FFU)),
                   "value over 1023 clamped");
        TEST_CHECK("S5-U09", ((UW)S5_REQ_SAFE_STOP == s5_request_pack(NULL)),
                   "pack(NULL) gives a SAFE_STOP word");
        TEST_CHECK("S5-U09", (E_PAR == subsystem5_get_request(NULL)),
                   "get_request(NULL) is E_PAR");
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
            90U, 17U, 90U, PASS_MAX_CM, 90U, 50U, 4U, 120U}; /* F1: shift 170 */
        static const plan_t left_plan  = {PLAN_PASS_LEFT, 170U};
        static const plan_t right_plan = {PLAN_PASS_RIGHT, 170U};
        static const plan_t stop_plan  = {PLAN_STOP, 0U};
        static const plan_t cont_plan  = {PLAN_CONTINUE, 0U};
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
                   "left pass, shift 170: L90, F17, R90, F up to 80 watched, "
                   "R90, F50 to line, F4, L up to 120");
        TEST_CHECK("S5-U35", b_right_ok,
                   "right pass: the mirror (R90, F17, L90, F up to 80, L90, "
                   "F50, F4, R up to 120)");
        TEST_CHECK("S5-U35", b_line_ok,
                   "only F3 (step 6) and T4 (step 8) stop on the line");
        TEST_CHECK("S5-U35", b_all_ok,
                   "only F3 waits for BOTH sensors to cross the line");
        TEST_CHECK(
            "S5-U35", b_flags_ok,
            "F2 (step 4) is the watched pass; T3 (step 5) looks ahead after");
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

    /* S5-U36: "passed the box" detector for the watched pass (F2) */
    {
        side_pass_t st = {FALSE, 0U};
        UW          index;
        BOOL        b_early = FALSE;
        BOOL        b_last  = FALSE;

        /* Near once, then SIDE_CLEAR_COUNT clear readings (OK far and
         * NO_ECHO alternating): passed only on the last one */
        (void)side_pass_update(&st, ULTRASONIC_OK, 300U, 700U);
        for (index = 1U; index <= SIDE_CLEAR_COUNT; index++)
        {
            b_last = side_pass_update(
                &st, (0U == (index & 1U)) ? ULTRASONIC_OK : ULTRASONIC_NO_ECHO,
                800U, 700U);
            if (b_last && (index < SIDE_CLEAR_COUNT))
            {
                b_early = TRUE;
            }
        }
        TEST_CHECK("S5-U36", (!b_early && b_last),
                   "near 300, then passed only after SIDE_CLEAR_COUNT (10) "
                   "clear readings");

        st.b_seen    = FALSE;
        st.clear_run = 0U;
        b_last       = FALSE;
        for (index = 0U; index < (2U * SIDE_CLEAR_COUNT); index++)
        {
            b_last =
                b_last || side_pass_update(&st, ULTRASONIC_TOO_FAR, 0U, 700U);
        }
        TEST_CHECK("S5-U36", !b_last,
                   "clear readings without seeing the box first: not passed");

        st.b_seen    = FALSE;
        st.clear_run = 0U;
        (void)side_pass_update(&st, ULTRASONIC_OK, 300U, 700U);
        for (index = 1U; index < SIDE_CLEAR_COUNT; index++)
        {
            (void)side_pass_update(&st, ULTRASONIC_OK, 800U, 700U);
        }
        TEST_CHECK("S5-U36",
                   (!side_pass_update(&st, ULTRASONIC_OK, 300U, 700U) &&
                    (0U == st.clear_run)),
                   "near again just before passing resets the clear count");

        st.b_seen    = FALSE;
        st.clear_run = 0U;
        (void)side_pass_update(&st, ULTRASONIC_OK, 300U, 700U);
        (void)side_pass_update(&st, ULTRASONIC_TOO_NEAR, 0U, 700U);
        (void)side_pass_update(&st, ULTRASONIC_BUSY, 0U, 700U);
        TEST_CHECK("S5-U36", (0U == st.clear_run),
                   "TOO_NEAR and BUSY change nothing (NFR5-5)");
        TEST_CHECK("S5-U36", !side_pass_update(NULL, ULTRASONIC_OK, 800U, 700U),
                   "NULL state is FALSE");
    }

    /* S5-U37: watched-pass distances */
    {
        obstacle_profile_t prof  = {TRUE, 190U, 80U,   185,
                                    -88,  205U, 9999U, 9999U};
        plan_t             left  = {PLAN_PASS_LEFT, 135U};
        plan_t             right = {PLAN_PASS_RIGHT, 135U};
        plan_t             stop  = {PLAN_STOP, 0U};

        TEST_CHECK(
            "S5-U37", (226U == bypass_side_gap_mm(&left, &prof)),
            "side gap, left pass: F1 140 + pivots 100 - left edge 14 = 226 mm");
        TEST_CHECK("S5-U37",
                   ((50U == bypass_side_gap_mm(&right, &prof)) &&
                    (0U == bypass_side_gap_mm(&stop, &prof)) &&
                    (0U == bypass_side_gap_mm(NULL, &prof))),
                   "side gap at least 50 mm; not a pass or NULL: 0");
        TEST_CHECK("S5-U37",
                   ((710U == bypass_side_near_mm(226U)) &&
                    (SIDE_NEAR_MAX_MM == bypass_side_near_mm(400U))),
                   "beside under 226 / sin 20 + 50 = 710 mm, at most 1000");
        TEST_CHECK("S5-U37", (35U == bypass_pass_plan_cm(&prof)),
                   "never seen: front 185 - 100 + 60 deep + 150 tail + 50 = "
                   "345 -> 35 cm");
        prof.closest_x_mm = 900;
        TEST_CHECK("S5-U37",
                   ((PASS_MAX_CM == bypass_pass_plan_cm(&prof)) &&
                    (10U == bypass_pass_plan_cm(NULL))),
                   "plan at most PASS_MAX_CM; NULL: 10 cm");
        TEST_CHECK("S5-U37", (43U == bypass_after_clear_cm(226U)),
                   "after passing: 226 x cot 60 + 150 + 50 + 100 sweep = 430 "
                   "-> 43 cm on");
        TEST_CHECK("S5-U37", (426U == bypass_probe_mm(226U)),
                   "after T3, box still ahead if under 226 + 200 = 426 mm");
        TEST_CHECK("S5-U37",
                   ((50U == bypass_side_deg(PLAN_PASS_LEFT)) &&
                    (130U == bypass_side_deg(PLAN_PASS_RIGHT)) &&
                    (SERVO_ANGLE_CENTRE_DEG == bypass_side_deg(PLAN_STOP))),
                   "left pass watches 50 (box on the right), right pass 130");
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

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
