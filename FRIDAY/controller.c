/*
 * Filtered PID line follower for the uploaded LEGO-tyre controller.
 * Only controller.c changes. No encoder motor-speed PI or fixed turn schedule.
 * Default RACE_PROFILE4 matches the uploaded selection; names remain targets,
 * not measured real lap times. PWM values/caps/ramp rates are retained so the
 * effect of line control can be compared at the same requested effort.
 *
 * Profile   normal/bar PWM   KP   KD   KI_Q8   motor cap
 *   16         50/35          4    2     2         80
 *   14         55/40          5    2     2         90
 *   12         63/45          6    3     2        100
 *   10         96/55          9    4     2        125
 *    8        125/65         11    6     2        140
 *    6        140/75         11    6     2        150
 *    4        150/90         11    6     2        150
 *
 * LINE PID AT5ms
 * P uses the fractional centroid with a small soft band(0.25weightunit=~1mm at
 * assumed8mm sensor pitch). D filters the error delta by1/2 and caps its effort
 * at24PWM. This reduces noise/chatter without adding P filtering or steering
 * slew delay. KI_Q8=2 means2/256PWM/error/sample, not2PWM/error/sample.
 * I is a small leaky straight-bias trim: only |raw error|<=1.5units and
 * |filtered delta|<=1unit integrate. It leaks with ~1.28s time constant, is
 * bounded+/-6PWM and has conditional antiwindup at actual steering saturation.
 * Bars hold last P, suppress D and leak I; loss of line slows then commands0
 * after200ms and clears I. Reacquisition starts with zero derivative.
 *
 * TUNING: integral does NOT cure oscillation. First set KI_Q8=0 if diagnosing;
 * tune P/D, then restore2 only for consistent bias. Lower KP by1 for slow
 * weaving/over-correction; increase KD by1..2 for damping. If high-frequency
 * chatter grows, lower KD and check sensor height/threshold/loose mounts.
 * Increase KI_Q8 slowly only for persistent straight bias after wheel alignment
 * checks. Never use more I to chase bends/slip. Lower base PWM for entry slip.
 * At150normalPWM/cap150 any steering correction MUST reduce the common PWM;
 * no PID can keep both motors at150while also creating a speed difference.
 * debug_normal_ticks/full_pwm_ticks quantify normal tracking time with common
 * PWM within1ofrequest; these measure commanded PWM, not physical speed.
 * debug_error_q8/derivative_q8 and debug_pid_p/d/i_q8 use value/256units.
 * debug_integral_q16 uses value/65536PWM. Common-PWM headroom flags are retained.
 *
 * WIRING: the user confirmed the uploaded hardware direction. Its internal
 * minus mix plus final swap gives effective left=common+correction. This file
 * preserves that effective hardware mix using canonical plus/minus once.
 * PHYSICAL_SWAP_OUTPUTS=0is therefore intentional. Native simulation uses the
 * same canonical direction, fixing the uploaded simulator-only inversion.
 * Set swap1only if the physical wiring is deliberately changed/verified.
 *
 * Same30.4mm nominal tyres,100mm wheel-centre track(50mm per side),56mm sensor
 * lead assuming COM on axle. Python: radius.0152,car_radius.050,car_length.112.
 * Sensor pitch8mm/mass.2kg remain assumptions. The launcher reads metadata;
 * changing C dimensions alone cannot alter Python physics.5ms rate is fixed.
 * The real PD racer was oscillating and capped around8.5s; the ideal simulator
 * does not reproduce that severity. PID improvements below are model evidence,
 * not a promise of a faster physical lap. PWM remains<=150, never255; caps do
 * not certify winding current/temperature on6V motors with nominal7.2V supply.
 */
#include "racer.h"
#include <stdint.h>
#ifndef RACE_PROFILE
#define RACE_PROFILE 8
#endif
#if RACE_PROFILE == 16
#define ACCEL_PWM 50
#define DECEL_PWM 35
#define KP 4
#define KD 2
#define KI_Q8 2
#define MOTOR_PWM_MAX 80
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8
#elif RACE_PROFILE == 14
#define ACCEL_PWM 55
#define DECEL_PWM 40
#define KP 5
#define KD 2
#define KI_Q8 2
#define MOTOR_PWM_MAX 90
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8
#elif RACE_PROFILE == 12
#define ACCEL_PWM 63
#define DECEL_PWM 45
#define KP 6
#define KD 3
#define KI_Q8 2
#define MOTOR_PWM_MAX 100
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8
#elif RACE_PROFILE == 10	// runs around 10.2s
#define ACCEL_PWM 96
#define DECEL_PWM 55
#define KP 9
#define KD 4
#define KI_Q8 2
#define MOTOR_PWM_MAX 125
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8
#elif RACE_PROFILE == 8		// runs an 8.5s on low battery
#define ACCEL_PWM 125           // originally 125
#define DECEL_PWM 125
#define KP 11
#define KD 6
#define KI_Q8 2
#define MOTOR_PWM_MAX 150       // originally 140
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8
#elif RACE_PROFILE == 6
#define ACCEL_PWM 140
#define DECEL_PWM 75
#define KP 11
#define KD 6
#define KI_Q8 2
#define MOTOR_PWM_MAX 150
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8
#elif RACE_PROFILE == 4		// ran 8s on super low battery
#define ACCEL_PWM 150
#define DECEL_PWM 150
#define KP 8            // originally 11
#define KD 10            // originally 6
#define KI_Q8 2
#define MOTOR_PWM_MAX 150
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8
#else
#error "RACE_PROFILE must be 16, 14, 12, 10, 8, 6 or 4"
#endif
#ifndef PID_KP
#define PID_KP KP
#endif
#ifndef PID_KD
#define PID_KD KD
#endif
#ifndef PID_KI_Q8
#define PID_KI_Q8 KI_Q8
#endif
#ifndef ERROR_DEADBAND_Q8
#define ERROR_DEADBAND_Q8 64
#endif
#ifndef D_FILTER_DIV
#define D_FILTER_DIV 2
#endif
#ifndef D_LIMIT_PWM
#define D_LIMIT_PWM 24
#endif
#define I_ZONE_Q8 384
#define I_D_GATE_Q8 256
#define I_LIMIT_PWM 6
#define I_LEAK_DIV 256
#ifndef PHYSICAL_SWAP_OUTPUTS
#define PHYSICAL_SWAP_OUTPUTS 0
#endif
// A marker must also reach both outer sensor pairs; an off-centre
// five-bit curve pattern remains a line measurement rather than a marker.
#define CROSSBAR_THRESHOLD 5
#define LOST_LINE_PWM 35
#define LOST_STOP_TICKS 40
#ifndef RACER_SIMULATOR
#ifdef __AVR__
#define RACER_SIMULATOR 0
#else
#define RACER_SIMULATOR 1
#endif
#endif
#ifndef HAS_START_FINISH_STRIPE
#define HAS_START_FINISH_STRIPE (!RACER_SIMULATOR)
#endif
// Geometry metadata feeds the simulator launcher; it does not automatically
// recalibrate PID gains or Python dynamics.
#define WHEEL_DIAMETER_TENTHS_MM 304
#define WHEEL_TRACK_MM 100
#define SENSOR_LEAD_MM 56
#define SENSOR_SPACING_MM 8
#define CHASSIS_MASS_GRAMS 200

#if MOTOR_PWM_MAX > 150 || ACCEL_PWM > MOTOR_PWM_MAX || DECEL_PWM > ACCEL_PWM
#error "Invalid PWM limits: require DECEL_PWM <= ACCEL_PWM <= MOTOR_PWM_MAX <= 150"
#endif
#if BASE_RISE_STEP <= 0 || BASE_FALL_STEP <= 0 || LOST_STOP_TICKS <= 0
#error "PWM ramp steps and lost-line timeout must be positive"
#endif

#if D_FILTER_DIV < 1 || I_LEAK_DIV < 1 || PID_KI_Q8 < 0 || PID_KP < 0 || PID_KD < 0
#error "Invalid PID gain/filter setting"
#endif
#if RACER_SIMULATOR != 0 && RACER_SIMULATOR != 1
#error "RACER_SIMULATOR must be0or1"
#endif
#if PHYSICAL_SWAP_OUTPUTS != 0 && PHYSICAL_SWAP_OUTPUTS != 1
#error "PHYSICAL_SWAP_OUTPUTS must be0or1"
#endif

const int16_t debug_wheel_diameter_tenths_mm = WHEEL_DIAMETER_TENTHS_MM;
const int16_t debug_wheel_track_mm = WHEEL_TRACK_MM;
const int16_t debug_sensor_lead_mm = SENSOR_LEAD_MM;
const int16_t debug_sensor_spacing_mm = SENSOR_SPACING_MM;
const int16_t debug_chassis_mass_grams = CHASSIS_MASS_GRAMS;
const int16_t debug_has_start_finish_stripe = HAS_START_FINISH_STRIPE;
const int16_t debug_simulator_output_adapter = RACER_SIMULATOR;
const int16_t debug_pid_kp = PID_KP, debug_pid_kd = PID_KD;
const int16_t debug_pid_ki_q8 = PID_KI_Q8;
const int16_t debug_physical_swap_outputs = PHYSICAL_SWAP_OUTPUTS;
volatile int16_t debug_selected_profile = RACE_PROFILE;
volatile int16_t debug_motor_pwm_limit = MOTOR_PWM_MAX;
volatile int16_t debug_line_error, debug_pd_derivative, debug_steering_effort;
volatile int16_t debug_speed_target, debug_base_pwm, debug_left_pwm, debug_right_pwm;
volatile int16_t debug_peak_abs_pwm;
// Legacy benchmark compatibility: there is no distance-based turn state.
volatile uint8_t debug_motion_state, debug_turn_number, debug_crossbar;
volatile uint8_t debug_line_valid, debug_steering_limited, debug_speed_reduced;
volatile uint16_t debug_lost_ticks, debug_crossbar_count;
static int16_t base_pwm;
static int16_t previous_error_q8, last_error_q8, filtered_d_q8;
static int32_t integral_q16;
static uint16_t lost_ticks;
static uint8_t history_valid, previous_bar;

volatile int16_t debug_error_q8, debug_derivative_q8;
volatile int16_t debug_pid_p_q8, debug_pid_d_q8, debug_pid_i_q8;
volatile int32_t debug_integral_q16;
volatile uint8_t debug_integrating;
volatile uint32_t debug_normal_ticks, debug_full_pwm_ticks, debug_headroom_ticks;
static int16_t abs16(int16_t x) {
    return x < 0 ? -x : x;
}
static int32_t clip32(int32_t x, int32_t lo, int32_t hi) {
    return x < lo ? lo : x > hi ? hi : x;
}
static int16_t rounded_q8(int32_t x) {
    return x >= 0 ? (int16_t)((x + 128) / 256) : -(int16_t)((-x + 128) / 256);
}
uint8_t activeLEDs(uint8_t bits) {
    uint8_t n = 0;
    while (bits) {
        n += bits & 1U;
        bits >>= 1;
    }
    return n;
}
// Q8 preserves fractional centroids from non-contiguous/noisy bit patterns.
static int16_t position_q8(uint8_t bits) {
    int16_t sum = 0;
    uint8_t n = 0, i;
    for (i = 0; i < 8; i++)
        if (bits & (1U << i)) {
            sum += (int16_t)(2 * i) - 7;
            n++;
        }
    return n ? (int16_t)((int32_t)sum * 256 / n) : last_error_q8;
}
static int16_t deadband(int16_t e) {
    if (e > ERROR_DEADBAND_Q8)
        return e - ERROR_DEADBAND_Q8;
    if (e < -ERROR_DEADBAND_Q8)
        return e + ERROR_DEADBAND_Q8;
    return 0;
}
// Anti-windup uses this pre-rounding feasible command; integer PWM rounding
// must not be mistaken for actuator saturation. Common-PWM reduction is also
// distinct from steering saturation and must not disable bias correction.
static int32_t feasible_q8(int32_t raw_q8, int16_t limit) {
    return clip32(raw_q8, -(int32_t)limit * 256L, (int32_t)limit * 256L);
}

void racer_init(void) {
    base_pwm = 0;
    lost_ticks = 0;
    history_valid = previous_bar = 0;
    previous_error_q8 = last_error_q8 = filtered_d_q8 = 0;
    integral_q16 = 0;
    debug_line_error = debug_pd_derivative = debug_steering_effort = 0;
    debug_speed_target = debug_base_pwm = debug_left_pwm = debug_right_pwm = debug_peak_abs_pwm = 0;
    debug_motion_state = debug_turn_number = debug_crossbar = debug_line_valid = 0;
    debug_steering_limited = debug_speed_reduced = 0;
    debug_lost_ticks = debug_crossbar_count = 0;
    debug_error_q8 = debug_derivative_q8 = debug_pid_p_q8 = debug_pid_d_q8 = debug_pid_i_q8 = 0;
    debug_integral_q16 = 0;
    debug_integrating = 0;
    debug_normal_ticks = debug_full_pwm_ticks = debug_headroom_ticks = 0;
}
void racer_control_step(void) {
    uint8_t sensors = read_sensors(), bar = activeLEDs(sensors) >= CROSSBAR_THRESHOLD &&
                                            (sensors & 0x03U) && (sensors & 0xc0U);
    uint8_t valid = sensors && !bar, integrate = 0;
    int16_t raw_error = last_error_q8, error = last_error_q8, limit, correction, common, left,
            right;
    int16_t request = bar ? DECEL_PWM : ACCEL_PWM;
    int32_t p, d, candidate, raw_q8, leaked, applied_q8;
    if (bar && !previous_bar && debug_crossbar_count < UINT16_MAX)
        debug_crossbar_count++;
    previous_bar = bar;
    if (sensors)
        lost_ticks = 0;
    else if (lost_ticks < UINT16_MAX)
        lost_ticks++;
    if (valid) {
        raw_error = position_q8(sensors);
        error = deadband(raw_error);
        if (history_valid)
            filtered_d_q8 += (error - previous_error_q8 - filtered_d_q8) / D_FILTER_DIV;
        else
            filtered_d_q8 = 0;
        previous_error_q8 = last_error_q8 = error;
        history_valid = 1;
    } else {
        history_valid = 0;
        filtered_d_q8 = 0;
    }
    if (!sensors && request > LOST_LINE_PWM)
        request = LOST_LINE_PWM;
    if (lost_ticks >= LOST_STOP_TICKS)
        request = 0;
    if (base_pwm < request)
        base_pwm = (int16_t)clip32(base_pwm + BASE_RISE_STEP, 0, request);
    else if (base_pwm > request)
        base_pwm = (int16_t)clip32(base_pwm - BASE_FALL_STEP, request, MOTOR_PWM_MAX);
    // D is a low-pass-filtered per-sample delta (the5ms dt is folded in KD).
    // No derivative kick on a marker or first line reacquisition.
    p = (int32_t)PID_KP * error;
    d = clip32((int32_t)PID_KD * filtered_d_q8, -D_LIMIT_PWM * 256L, D_LIMIT_PWM * 256L);
    // Leaky bias trim: do not integrate normal bend offset, large transients,
    // broad markers or a missing line. The output remains within +/-6 PWM.
    leaked = integral_q16 - integral_q16 / I_LEAK_DIV;
    candidate = leaked;
    if (PID_KI_Q8 > 0 && valid && abs16(raw_error) <= I_ZONE_Q8 &&
        abs16(filtered_d_q8) <= I_D_GATE_Q8) {
        integrate = 1;
        candidate += (int32_t)PID_KI_Q8 * error;
    }
    candidate = clip32(candidate, -I_LIMIT_PWM * 65536L, I_LIMIT_PWM * 65536L);
    limit = base_pwm;
    if (limit > MOTOR_PWM_MAX / 2)
        limit = MOTOR_PWM_MAX / 2;
    raw_q8 = p + d + candidate / 256;
    applied_q8 = feasible_q8(raw_q8, limit);
    correction = rounded_q8(applied_q8);
    if (integrate && ((raw_q8 > applied_q8 && error > 0) || (raw_q8 < applied_q8 && error < 0))) {
        candidate = leaked;
        integrate = 0;
    }
    integral_q16 = candidate;
    raw_q8 = p + d + integral_q16 / 256;
    correction = rounded_q8(feasible_q8(raw_q8, limit));
    // Both outputs must remain0..cap. At request==cap, any nonzero steering
    // necessarily reduces common effort; PID cannot bypass that constraint.
    common = base_pwm;
    if (common > MOTOR_PWM_MAX - abs16(correction))
        common = MOTOR_PWM_MAX - abs16(correction);
    left = common + correction;
    right = common - correction;
    if (lost_ticks >= LOST_STOP_TICKS) {
        left = right = common = base_pwm = correction = 0;
        integral_q16 = 0;
        raw_q8 = p = d = 0;
        filtered_d_q8 = 0;
        integrate = 0;
    }
    debug_line_error = rounded_q8(error);
    debug_pd_derivative = rounded_q8(filtered_d_q8);
    debug_steering_effort = correction;
    debug_speed_target = request;
    debug_base_pwm = common;
    debug_left_pwm = left;
    debug_right_pwm = right;
    debug_crossbar = bar;
    debug_line_valid = valid;
    debug_lost_ticks = lost_ticks;
    debug_steering_limited = (raw_q8 > limit * 256L || raw_q8 < -limit * 256L);
    debug_speed_reduced = common < base_pwm;
    debug_error_q8 = error;
    debug_derivative_q8 = filtered_d_q8;
    debug_pid_p_q8 = (int16_t)p;
    debug_pid_d_q8 = (int16_t)d;
    debug_pid_i_q8 = (int16_t)(integral_q16 / 256);
    debug_integral_q16 = integral_q16;
    debug_integrating = integrate;
    if (valid && debug_normal_ticks < UINT32_MAX) {
        debug_normal_ticks++;
        if (common >= ACCEL_PWM - 1 && debug_full_pwm_ticks < UINT32_MAX)
            debug_full_pwm_ticks++;
    }
    if (valid && debug_speed_reduced && debug_headroom_ticks < UINT32_MAX)
        debug_headroom_ticks++;
    if (left > debug_peak_abs_pwm)
        debug_peak_abs_pwm = left;
    if (right > debug_peak_abs_pwm)
        debug_peak_abs_pwm = right;
#if !RACER_SIMULATOR && PHYSICAL_SWAP_OUTPUTS
    set_speeds(right, left);
#else
    set_speeds(left, right);
#endif
}
