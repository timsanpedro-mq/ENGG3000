// DEMO VERSION: 7.2
// Built on Demo 6.1 (crossbar-selected turn plan + encoder distance + translational/rotational PI).
//
// 7.0: whole-lap track plan, braking on straights from encoder distance, turn feedforward from the
//      planned radius, PD line steering, wheel-speed feedforward (6.1's effort cap removed).
// 7.1: red start/finish line can't be mistaken for the T1 crossbar; used as a distance resync.
// 7.2: stability. Same pace (flying laps ~4.97 s in the sim), much more tolerant of real-car
//      differences:
//      - Sensor clean-up: only the block of adjacent lit sensors nearest the last line position
//        is used, so a flickering sensor can't jerk the steering.
//      - Crossbar must be seen on 2 consecutive readings: a flicker next to the line can't fake
//        a crossbar and start a turn early.
//      - Slow left/right motor-mismatch trim (KROTI_SLOW), learned on straights only, so a
//        weaker motor no longer pulls the car off the line. Handles 15% mismatch either side.
//      - Encoder self-calibration: the known distance between turn crossbars corrects the
//        distance scale (and speeds) each time a crossbar is seen. Handles wheel-size errors
//        of -8% to +8%.
//      - Fixed: 7.1 rounded each tick's encoder distance down (~2% under-count).
//      - Re-tuned gains/speeds for the above.
//
// Simulator results (this folder's racer.py: 15 mm wheels, 0.2 kg), 7.1 -> 7.2:
//  - Flying laps 4.96 -> ~4.97 s, first lap ~5.17 s.
//  - 28 stress cases: 20 -> 28 pass (start positions, red line visible, motor mismatch 8%/15%,
//    control delay, sensor flicker/dropouts, battery +/-15-20%, wheel size +/-8%, sensor
//    distance, mass, friction, combined).
//  - Sensor flicker over 20 random seeds: 7.1 failed up to 17/20, 7.2 0/20.
//  - Grip acceleration (forward/braking + cornering, 50 ms average) ~5.1 m/s^2 peak.
//
// Physical car: check WHEEL_DIAMETER_MM, HALF_TRACK_MM and the sensor distance. Place the car
// facing the home straight with its sensors on, or up to ~300 mm behind, the red start line.

#include "racer.h"
#include <stdint.h>

// ---------------- Speeds (mm/s) ----------------
#define V_STRAIGHT 1685   // Top speed on straights
#define V_T1A 740   // T1 chicane, R100 right
#define V_T1B 756   // T1 chicane, R100 left
#define V_T2 975   // T2, R200
#define V_T3 988   // T3, R200
#define V_T4A 885   // T4 S-bend, R150 right
#define V_T4B 872   // T4 S-bend, R150 left
#define V_T5A 828   // T5, R150 semicircle
#define V_T5B 800   // T5, R100 left
#define V_T5C 911   // T5, R150 right

#define ACCEL 4003  // mm/s^2 ramp-up limit
#define DECEL 3464  // mm/s^2 used to plan (and limit) braking
#define BRAKE_MARGIN_MM 40  // Finish braking this far before the slower section

// ---------------- Turn feedforward ----------------
#define CB_OFFSET_MM 20  // Car centre is this far before the arc when the crossbar is seen
#define FF_PCT 84  // % of the geometric turn feedforward applied
#define FF_LEAD_MM 18  // Switch to the next radius's feedforward this far early
#define FF_SLEW 28  // Max feedforward change per tick (x16 counts/tick)

// ---------------- Line steering (PD) ----------------
#define LKP 4  // (counts/tick x16) per sensor-weight unit of error
#define LKD 13  // (counts/tick x16) per unit change in error over D_WINDOW ticks
#define D_WINDOW 2  // Ticks the derivative is measured over (max 16)
#define LOST_ERR 17  // Error used when no sensor sees the line (beyond the +/-7 range)

// ---------------- Wheel speed loops ----------------
#define KFF 646  // Feedforward PWM per count/tick, x100
#define FRIC_PWM 7  // Feedforward PWM to overcome rolling friction
#define KTRP_NUM 26  // Translational PI (on x16 errors, >> 8)
#define KTRI_NUM 3
#define KROTP_NUM 18  // Rotational PI (on x16 errors, >> 8)
#define KROTI_NUM 1
#define INTEG_MAX 1260  // Integrator clamp (x16 count units)
#define ENC_AVG 3  // Encoder speed averaged over this many ticks (max 8)
#define PWM_SLEW 48  // Max PWM change per tick per wheel
#define BRAKE_PWM_MAX 0  // Most reverse PWM allowed (0 = no reverse; the driver's reverse has a dead band)

// ---------------- Crossbar detection ----------------
#define KROTI_SLOW     56     // Slow rotational integral (>> 12), learned on straights only:
#define IROT_SLOW_MAX  48344  //   trims left/right motor mismatch without reacting to every bend

// ---------------- Robustness ----------------
#define SENS_FILTER  1  // Use only the block of adjacent lit sensors nearest the last line position
#define CB_DEBOUNCE  2  // Readings a line-wide mark must persist for (rejects sensor flicker)
#define ENC_CAL      1  // Learn the encoder distance scale from the known crossbar spacing
#define CB_THRESHOLD 5  // Sensors lit at once that count as a crossbar
#define CB_WINDOW_MM 200  // Only accept a turn crossbar within this distance of where the plan expects it

// ---------------- Red start/finish line ----------------
// The red tape may read as white to the IR sensors. It sits ~390 mm before the T1 crossbar, so it
// is never taken as a turn: it is used as an extra distance reference instead.
#define START_LINE_MM 375  // Car-centre distance along the home straight when the sensors reach it
#define START_LINE_TOL 150  // A line-wide mark this close to START_LINE_MM is the start/finish line
#define START_SEARCH_MM 330  // From power-up, a line-wide mark within this travel is the start line

// ---------------- Geometry ----------------
#define WHEEL_DIAMETER_MM 30L   // Simulator: wheel_radius = 0.015 m. Measure the real wheel.
#define COUNTS_PER_REV    360L
#define HALF_TRACK_MM     40L   // Simulator: car_radius = 0.04 m (half the wheel spacing)
// Encoder counts per mm x10000 (= 360 / (pi * 30) = 3.818 for the simulator)
#define CPM_X10000 ((COUNTS_PER_REV * 10000L * 7L) / (22L * WHEEL_DIAMETER_MM))
#define MM_TO_COUNTS(mm) ((int32_t)(mm) * CPM_X10000 / 10000L)
#define COUNTS_TO_MM(c)  ((int32_t)(c) * 10000L / CPM_X10000)
// mm/s -> counts per 5 ms tick, x16
#define MMS_TO_V16(v) ((int32_t)(v) * CPM_X10000 * 16L / (200L * 10000L))

// ---------------- Track plan ----------------
// One lap as centre-line sections. radius 0 = straight. dir +1 = left wheel faster (right turn).
// crossbar = the section starts at a crossbar and is only entered when that crossbar is seen.
typedef struct {
    int16_t radius_mm;
    int8_t dir;
    int16_t len_mm;
    int8_t crossbar;
} Segment;

#define NUM_SEGS 14
static const Segment track[NUM_SEGS] = {
    {100, +1,  157, 1},   //  0 T1 chicane, first half
    {100, -1,  157, 0},   //  1    second half
    {  0,  0,  500, 0},   //  2
    {200, +1,  314, 1},   //  3 T2
    {  0,  0,  250, 0},   //  4
    {200, +1,  314, 1},   //  5 T3
    {  0,  0, 1150, 0},   //  6 back straight
    {150, +1,  236, 1},   //  7 T4 S-bend
    {150, -1,  236, 0},   //  8
    {  0,  0,  300, 0},   //  9
    {150, +1,  471, 1},   // 10 T5 semicircle
    {100, -1,  157, 0},   // 11
    {150, +1,  236, 0},   // 12
    {  0,  0,  800, 0},   // 13 home straight back to T1
};
#define START_SEG      13
#define START_DIST_MM START_LINE_MM   // Car placed with its sensors on the start/finish line

static const int8_t sensor_weight[8] = { -7, -5, -3, -1, 1, 3, 5, 7 };

static uint8_t seg;               // Current track section
static int32_t seg_dist;          // Encoder counts along it (negative just after a crossbar)
static int32_t v_cmd;             // Commanded forward speed, mm/s
static int32_t ff_out;            // Applied turn feedforward (x16 counts/tick)
static int16_t err_hist[16];      // Line error history for the derivative (tenths)
static uint8_t hist_idx;
static int16_t last_line_err;     // Line error on the previous tick (tenths)
static int32_t i_straight, i_turn, i_turn_slow;
static uint8_t crossbar_passed;
static uint8_t cb_count;          // consecutive line-wide readings
static uint16_t dist_scale;       // Encoder distance scale, x1024 (learned from crossbar spacing)
static int32_t dist_frac;         // Fractional remainder of scaled distance
static int32_t travel_since_cb;   // Raw encoder counts since the last accepted turn crossbar
static int8_t last_cb_seg;        // Section of the last accepted turn crossbar (-1 = none yet)
static int32_t start_travel;      // Counts travelled since power-up, until the start line is found
static uint8_t start_pending;     // 1 until the start line is seen or START_SEARCH_MM is covered
static int16_t lhist[8], rhist[8];
static int16_t lsum, rsum;
static uint8_t enc_idx;
static int16_t lpwm_out, rpwm_out;

// Debugger-visible values
volatile uint8_t debug_seg;
volatile int16_t debug_v_cmd;

static uint8_t next_seg(uint8_t s) {
    return (s + 1 < NUM_SEGS) ? s + 1 : 0;
}

static int16_t seg_speed(uint8_t s) {
    switch (s) {
        case 0:  return V_T1A;
        case 1:  return V_T1B;
        case 3:  return V_T2;
        case 5:  return V_T3;
        case 7:  return V_T4A;
        case 8:  return V_T4B;
        case 10: return V_T5A;
        case 11: return V_T5B;
        case 12: return V_T5C;
        default: return V_STRAIGHT;
    }
}

// Integer square root (for the braking curve)
static int32_t isqrt32(int32_t x) {
    if (x <= 0) return 0;
    uint32_t r = 0, b = 1UL << 30, n = (uint32_t)x;
    while (b > n) b >>= 2;
    while (b) {
        if (n >= r + b) { n -= r + b; r = (r >> 1) + b; }
        else r >>= 1;
        b >>= 2;
    }
    return (int32_t)r;
}

static int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void racer_init(void) {
    seg = START_SEG;
    seg_dist = MM_TO_COUNTS(START_DIST_MM);
    v_cmd = 0;
    ff_out = 0;
    for (uint8_t i = 0; i < 16; i++) err_hist[i] = 0;
    hist_idx = 0;
    last_line_err = 0;
    i_straight = i_turn = i_turn_slow = 0;
    crossbar_passed = 0;
    cb_count = 0;
    dist_scale = 1024; dist_frac = 0; travel_since_cb = 0; last_cb_seg = -1;
    start_travel = 0;
    start_pending = 1;
    for (uint8_t i = 0; i < 8; i++) lhist[i] = rhist[i] = 0;
    lsum = rsum = 0;
    enc_idx = 0;
    lpwm_out = rpwm_out = 0;
}

void racer_control_step(void) {
    int16_t lenc = read_left_enc();
    int16_t renc = read_right_enc();
    uint8_t sensors = read_sensors();

    // Line position
    int16_t error_sum = 0;
    uint8_t active = 0;
    if (SENS_FILTER) {
        // Split the lit sensors into blocks of adjacent sensors and keep the block whose centre is
        // nearest the last line position; isolated flickering sensors are ignored.
        int16_t best_sum = 0; uint8_t best_cnt = 0; int16_t best_dist = 32767;
        uint8_t i = 0;
        while (i < 8) {
            if (sensors & (1 << i)) {
                int16_t sum = 0; uint8_t cnt = 0;
                while (i < 8 && (sensors & (1 << i))) { sum += sensor_weight[i]; cnt++; i++; }
                int16_t centre = (sum * 10) / cnt;
                int16_t dist = centre - last_line_err; if (dist < 0) dist = -dist;
                if (cnt > best_cnt + 1 || (cnt + 1 >= best_cnt && dist < best_dist) ) {
                    best_sum = sum; best_cnt = cnt; best_dist = dist;
                }
            } else i++;
        }
        error_sum = best_sum; active = best_cnt;
    } else {
        for (uint8_t i = 0; i < 8; i++) {
            if (sensors & (1 << i)) {
                error_sum += sensor_weight[i];
                active++;
            }
        }
    }

    uint8_t wide = (active >= CB_THRESHOLD);
    if (!wide) cb_count = 0; else if (cb_count < 255) cb_count++;
    uint8_t on_crossbar = (cb_count >= CB_DEBOUNCE);
    uint8_t new_mark = on_crossbar && !crossbar_passed;
    crossbar_passed = on_crossbar;

    // Start/finish line: at power-up the first line-wide mark is the start line; on later laps a
    // mark near its planned position is the start line. Either way, resync distance to it.
    if (start_pending) {
        start_travel += ((lenc < 0 ? -lenc : lenc) + (renc < 0 ? -renc : renc)) / 2;
        if (start_travel > MM_TO_COUNTS(START_SEARCH_MM)) start_pending = 0;
    }
    if (new_mark && seg == START_SEG &&
        (start_pending ||
         (seg_dist > MM_TO_COUNTS(START_LINE_MM - START_LINE_TOL) &&
          seg_dist < MM_TO_COUNTS(START_LINE_MM + START_LINE_TOL)))) {
        seg_dist = MM_TO_COUNTS(START_LINE_MM);
        start_pending = 0;
        new_mark = 0;
    }

    // Crossbar on a straight, close to where the plan expects it: jump to the start of the turn
    if (new_mark && track[seg].radius_mm == 0) {
        uint8_t nxt = next_seg(seg);
        if (track[nxt].crossbar &&
            seg_dist >= MM_TO_COUNTS(track[seg].len_mm - CB_WINDOW_MM)) {
            // Calibrate: planned distance since the previous turn crossbar vs encoder counts
            if (ENC_CAL && last_cb_seg >= 0 && travel_since_cb > 0) {
                int32_t plan_mm = 0;
                uint8_t s2 = (uint8_t)last_cb_seg;
                while (s2 != nxt) { plan_mm += track[s2].len_mm; s2 = next_seg(s2); }
                int32_t scale = MM_TO_COUNTS(plan_mm) * 2048L / travel_since_cb;   // x1024
                if (scale > 900 && scale < 1150)
                    dist_scale = (uint16_t)((3L * dist_scale + scale) / 4);
            }
            last_cb_seg = (int8_t)nxt;
            travel_since_cb = 0;
            seg = nxt;
            seg_dist = -MM_TO_COUNTS(CB_OFFSET_MM);
        }
    }

    // Distance along the plan (average wheel movement = chassis-centre distance), scaled by the
    // learned encoder calibration
    {
        int32_t raw = ((lenc < 0 ? -lenc : lenc) + (renc < 0 ? -renc : renc));   // 2 x counts
        travel_since_cb += raw;
        int32_t acc = raw * (int32_t)dist_scale + dist_frac;                   // x2048
        seg_dist += acc >> 11;
        dist_frac = acc & 2047;
    }
    int32_t seg_len = MM_TO_COUNTS(track[seg].len_mm);
    if (seg_dist >= seg_len) {
        uint8_t nxt = next_seg(seg);
        // A turn that starts at a crossbar is only entered when the crossbar is seen
        if (!(track[seg].radius_mm == 0 && track[nxt].crossbar)) {
            seg_dist -= seg_len;
            seg = nxt;
        }
    }

    // Speed target: this section's speed, limited by braking for the next three sections
    int32_t v_target = seg_speed(seg);
    int32_t remain_mm = COUNTS_TO_MM(seg_len - seg_dist) - BRAKE_MARGIN_MM;
    if (remain_mm < 0) remain_mm = 0;
    uint8_t s = seg;
    for (uint8_t k = 0; k < 3; k++) {
        s = next_seg(s);
        int32_t vn = seg_speed(s);
        int32_t v_allow = isqrt32(vn * vn + 2L * DECEL * remain_mm);
        if (v_allow < v_target) v_target = v_allow;
        remain_mm += track[s].len_mm;
    }
    // Past the planned end of a straight without seeing the crossbar yet: hold the turn speed
    uint8_t nxt = next_seg(seg);
    if (track[seg].radius_mm == 0 && track[nxt].crossbar &&
        seg_dist >= seg_len - MM_TO_COUNTS(BRAKE_MARGIN_MM)) {
        if (seg_speed(nxt) < v_target) v_target = seg_speed(nxt);
    }

    // Ramp the commanded speed towards the target
    if (v_target > v_cmd + ACCEL / 200) v_cmd += ACCEL / 200;
    else if (v_target < v_cmd - DECEL / 200) v_cmd -= DECEL / 200;
    else v_cmd = v_target;
    int32_t v16 = MMS_TO_V16(v_cmd) * 1024L / dist_scale;   // counts/tick corrected by calibration

    // Turn feedforward from the planned radius, switched FF_LEAD_MM before the section boundary
    // (never early into a crossbar-triggered turn) and eased in by FF_SLEW
    uint8_t fseg = seg;
    if (!track[nxt].crossbar && seg_dist + MM_TO_COUNTS(FF_LEAD_MM) >= seg_len) fseg = nxt;
    int32_t ff16 = 0;
    if (track[fseg].radius_mm != 0 && (seg_dist >= 0 || fseg != seg)) {
        ff16 = v16 * HALF_TRACK_MM * track[fseg].dir * FF_PCT /
               ((int32_t)track[fseg].radius_mm * 100);
    }
    ff_out = clamp32(ff16, ff_out - FF_SLEW, ff_out + FF_SLEW);

    // Line PD (error in tenths; positive = line to the right = speed up the left wheel)
    int16_t line_err;
    if (active == 0) line_err = (last_line_err < 0) ? -LOST_ERR * 10 : LOST_ERR * 10;
    else if (on_crossbar) line_err = last_line_err;   // Average is meaningless on a crossbar
    else line_err = (error_sum * 10) / active;
    int16_t d_err = line_err - err_hist[hist_idx];
    err_hist[hist_idx] = line_err;
    if (++hist_idx >= D_WINDOW) hist_idx = 0;
    last_line_err = line_err;
    int32_t steer16 = ((int32_t)line_err * LKP + (int32_t)d_err * LKD) / 10;

    // Left/right wheel targets (counts/tick x16)
    int32_t wl16 = v16 + ff_out + steer16;
    int32_t wr16 = v16 - ff_out - steer16;

    // Measured wheel speeds, averaged over ENC_AVG ticks (x16)
    lsum += lenc - lhist[enc_idx];
    lhist[enc_idx] = lenc;
    rsum += renc - rhist[enc_idx];
    rhist[enc_idx] = renc;
    if (++enc_idx >= ENC_AVG) enc_idx = 0;
    int32_t errorL = wl16 - (int32_t)lsum * 16 / ENC_AVG;
    int32_t errorR = wr16 - (int32_t)rsum * 16 / ENC_AVG;

    // Sum isolates common translation; difference isolates rotation
    int32_t straight_error = errorL + errorR;
    int32_t turn_error = errorL - errorR;

    i_straight = clamp32(i_straight + straight_error, -INTEG_MAX, INTEG_MAX);
    i_turn = clamp32(i_turn + turn_error, -INTEG_MAX, INTEG_MAX);
    int32_t trans_out = (KTRP_NUM * straight_error + KTRI_NUM * i_straight) >> 8;
    int32_t rot_out = (KROTP_NUM * turn_error + KROTI_NUM * i_turn) >> 8;
    if (track[seg].radius_mm == 0 && ff_out == 0)   // learn motor mismatch on straights only
        i_turn_slow = clamp32(i_turn_slow + turn_error, -IROT_SLOW_MAX, IROT_SLOW_MAX);
    rot_out += (KROTI_SLOW * i_turn_slow) >> 12;

    // Feedforward + PI, then limit and slew each wheel
    int32_t lw_pwm = wl16 * KFF / 1600 + (wl16 > 0 ? FRIC_PWM : 0) + trans_out + rot_out;
    int32_t rw_pwm = wr16 * KFF / 1600 + (wr16 > 0 ? FRIC_PWM : 0) + trans_out - rot_out;
    lw_pwm = clamp32(lw_pwm, -BRAKE_PWM_MAX, 255);
    rw_pwm = clamp32(rw_pwm, -BRAKE_PWM_MAX, 255);
    lw_pwm = clamp32(lw_pwm, lpwm_out - PWM_SLEW, lpwm_out + PWM_SLEW);
    rw_pwm = clamp32(rw_pwm, rpwm_out - PWM_SLEW, rpwm_out + PWM_SLEW);
    lpwm_out = (int16_t)lw_pwm;
    rpwm_out = (int16_t)rw_pwm;

    debug_seg = seg;
    debug_v_cmd = (int16_t)v_cmd;

    set_speeds(lpwm_out, rpwm_out);
}
