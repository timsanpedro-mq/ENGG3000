/*
 * Crossbar-triggered braking / turn hold / acceleration with line PID throughout.
 * Only controller.c changes. The cancelled straight-only gain experiment is NOT
 * applied: profile4 retains KP11,KD6,KI_Q8=2,normal150,turn90 and cap150.
 * Default profile4 matches the uploaded selection; names remain target labels.
 * Other profile PID gains/PWM limits and the confirmed hardware direction remain.
 *
 * SPEED POLICY
 * - Keep the uploaded detector: at least5active sensors spanning both outer pairs.
 *   Require4clear samples(20ms) before another rising-edge marker event.
 * - A bend marker requests DECEL_PWM. BASE_FALL_STEP ramps the common demand
 *   down; the reduced request persists AFTER the bar until the whole bend ends.
 * - Prevent a centre-line crossing inside an S-bend from ending the slow section:
 *   require at least85%of that section's nominal encoder distance, at least250ms,
 *   then100ms of valid settled line(raw error<=1.5units,delta<=0.5unit/sample).
 * - Release to ACCEL_PWM and ramp up by BASE_RISE_STEP on the straight. PID
 *   steering stays active in braking, turn hold, acceleration and cruise.
 * - A new marker after enough section+straight travel starts the next bend even
 *   if the previous line-confidence exit did not complete; it cannot be ignored
 *   indefinitely by an old turn state. The slower effort is retained in that case.
 *
 * FIVE-SECTION COURSE / STRIPES
 * Encoder gate uses R100+R100 / R200 / R200 / R150+R150 /
 * R150semicircle+R100+R150. With30.4mm tyres, full sections are1184/1184/1184/
 * 1776/3256counts. This is a speed-planning guard, not programmed steering.
 * Remeasure loaded tyre rollout and360counts/rev. Gates need working encoders.
 * Native stock sensing excludes red; hardware HAS_START_FINISH_STRIPE defaults1.
 * Ignore the launch stripe within100mm of reset; after bend5 ignore the red
 * finish in its900..1500mm travel window from that marker. A missed stripe must
 * not consume the next bend marker. These windows assume the existing track.
 *
 * PROFILE4 TUNING
 * ACCEL_PWM150: normal straight request; DECEL_PWM90: sustained turn request.
 * BASE_FALL_STEP8/5ms: smaller means gentler/longer braking, not stronger braking.
 * BASE_RISE_STEP3/5ms: smaller means gentler/longer acceleration.
 * STRAIGHT_CONFIRM_TICKS20: increase toward30if entry still unsettled; decrease
 * cautiously if the car stays slow well into a straight. Keep the distance gate.
 * STRAIGHT_ERROR_Q8=384 and STRAIGHT_DELTA_Q8=128 define the exit confidence;
 * widening them releases earlier but may classify a curve as straight.
 * Turn too fast/slips: lower DECEL_PWM. Only raise it after repeatable grip tests.
 * More stable entries may initially cost lap time; profile names are not promises.
 *
 * PID / CAPS / GEOMETRY
 * PID coefficients/filter/soft band/leaky bounded I are the prior implementation.
 * Crossbars hold P, suppress D and leak I; line loss slows then commands0after
 * 200ms. The common-demand ramps precede steering allocation; steering can force
 * a faster reduction in actual common PWM to keep each motor within0..its cap.
 * At150request/cap any steering needs headroom. No command exceeds150oruses255.
 * PWM limits do not certify motor current/temperature on6V motors /7.2V supply.
 * Same30.4mm tyre,100mm wheel-centre track(50mm per side),56mm sensor lead(COM
 * assumed on axle). Pythonradius.0152,car_radius.050,car_length.112; pitch8mm and
 * mass.2kg are assumptions. PHYSICAL_SWAP_OUTPUTS=0preserves the confirmed
 * uploaded effective plus/minus mix. Encoders are NOT used for motor-speed PI.
 *
 * PROJECT ACCEL/DECEL EVIDENCE
 * debug_speed_phase:0cruise,1braking,2cornerhold,3accelerating.
 * debug_speed_target=request;debug_ramped_pwm=rate-limited demand;
 * debug_base_pwm=common command after steering headroom allocation.
 * debug_accelerating_ticks/braking_ticks count common-demand ramp samples.
 * debug_left/right_encoder are signed counts/5ms. debug_measured_speed_q8 is
 * a1/4filtered wheel-centre count/sample estimate; debug_measured_accel_q8 is
 * its per-sample delta. Divide by256for count units. With95.504mm circumference,
 * speed_m/s=counts*0.053058;acceleration_m/s^2=delta_counts*10.6116.
 * These measure wheel acceleration; slip can separate that from ground motion.
 * Use real logs/video to demonstrate physical acceleration/deceleration; model
 * tests verify control phases but cannot certify grip or real lap-time improvement.
 */
#include "racer.h"
#include <stdint.h>
#ifndef RACE_PROFILE
#define RACE_PROFILE 20             
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

/////////////////////////////////////////////////

// MAIN PROFILE  completed sub 8s consistently
#elif RACE_PROFILE == 10
#define ACCEL_PWM 120           // originally 96
#define DECEL_PWM 90            // originally 55
#define KP 20                    // originally 9         7
#define KD 50                    // originally 4         9
#define KI_Q8 3
#define MOTOR_PWM_MAX 135       // originally 125        set to 135
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8

/////////////////////////////////////////////////

/////////////////////////////////////////////////

// FASTEST PROFILE completed consistent sub 7s, around 6.7s
#elif RACE_PROFILE == 20
#define ACCEL_PWM 150
#define DECEL_PWM 110
#define KP 20                   
#define KD 50                    
#define KI_Q8 3                 
#define MOTOR_PWM_MAX 150       
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8

/////////////////////////////////////////////////

/////////////////////////////////////////////////

// EXPERIMENTAL PROFILE 
#elif RACE_PROFILE == 67
#define ACCEL_PWM 150
#define DECEL_PWM 120
#define KP 24                   
#define KD 70                    
#define KI_Q8 4                 
#define MOTOR_PWM_MAX 150       
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8

/////////////////////////////////////////////////

#elif RACE_PROFILE == 8
#define ACCEL_PWM 125
#define DECEL_PWM 65
#define KP 11
#define KD 6
#define KI_Q8 2
#define MOTOR_PWM_MAX 140
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
#elif RACE_PROFILE == 4
#define ACCEL_PWM 150
#define DECEL_PWM 90
#define KP 11
#define KD 6
#define KI_Q8 2
#define MOTOR_PWM_MAX 150
#define BASE_RISE_STEP 3
#define BASE_FALL_STEP 8
#else
#error "RACE_PROFILE must be 16, 14, 12, 10, 8, 6, 4, 20 or 67"
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
// Crossbars initiate a lower-speed section; stable line following releases it.
#define TURN_MIN_HOLD_TICKS 50U    // 250ms minimum after a bend marker
#define STRAIGHT_CONFIRM_TICKS 20U // 100ms of settled line before accelerating
#define STRAIGHT_ERROR_Q8 384      // raw centroid within1.5weightunits
#define STRAIGHT_DELTA_Q8 128      // filtered delta within0.5unit/sample
#define MARKER_CLEAR_TICKS 4U      // 20ms of clear sensors before another edge
// Same five entry-marker track used by racer.py. Fullwidth red stripe is an
// additional launch/finish marker on the physical track, not a turn marker.
#define TURN_MIN_PROGRESS_PERCENT 85L
#define ENCODER_COUNTS_PER_REV 360L
#define QUARTER_COUNTS(r)                                                                          \
    (((r) * ENCODER_COUNTS_PER_REV * 10L + WHEEL_DIAMETER_TENTHS_MM) /                             \
     (2L * WHEEL_DIAMETER_TENTHS_MM))

#define FINISH_MIN_MM 900L
#define FINISH_MAX_MM 1500L
#define WHEEL_CIRCUMFERENCE_UM ((WHEEL_DIAMETER_TENTHS_MM * 314159L + 500L) / 1000L)
#define MM_TO_COUNTS(mm)                                                                           \
    (((int32_t)(mm) * ENCODER_COUNTS_PER_REV * 1000L + WHEEL_CIRCUMFERENCE_UM / 2L) /              \
     WHEEL_CIRCUMFERENCE_UM)
#define PHASE_CRUISE 0U
#define PHASE_BRAKING 1U
#define PHASE_CORNER 2U
#define PHASE_ACCELERATING 3U
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
static const int16_t turn_section_counts[5] = {2L * QUARTER_COUNTS(100L), QUARTER_COUNTS(200L),
                                               QUARTER_COUNTS(200L), 2L * QUARTER_COUNTS(150L),
                                               3L * QUARTER_COUNTS(150L) + QUARTER_COUNTS(100L)};

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

#if STRAIGHT_CONFIRM_TICKS < 1 || TURN_MIN_HOLD_TICKS < 1 || MARKER_CLEAR_TICKS < 1
#error "Turn/marker confirmation settings must be positive"
#endif
#if TURN_MIN_PROGRESS_PERCENT <= 0 || TURN_MIN_PROGRESS_PERCENT > 100
#error "Turn progress percentage must be in1..100"
#endif
#if WHEEL_DIAMETER_TENTHS_MM <= 0 || ENCODER_COUNTS_PER_REV <= 0 || FINISH_MIN_MM >= FINISH_MAX_MM
#error "Invalid odometry or finish-marker calibration"
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
static uint8_t turn_active, active_bend, next_bend, launch_pending, finish_pending, clear_ticks;
static uint16_t turn_ticks, straight_ticks;
static int32_t launch_distance_twice, finish_distance_twice, turn_distance_twice;
static int32_t measured_speed_q8;
volatile uint8_t debug_speed_phase, debug_turn_active, debug_launch_crossbar_ignored;
volatile uint16_t debug_turn_ticks, debug_straight_ticks, debug_turn_entries, debug_turn_exits;
volatile int32_t debug_turn_distance_counts, debug_turn_min_counts;
volatile int16_t debug_ramped_pwm;
volatile uint16_t debug_finish_count;
volatile uint32_t debug_accelerating_ticks, debug_braking_ticks;
volatile int16_t debug_left_encoder, debug_right_encoder;
volatile int16_t debug_measured_speed_q8, debug_measured_accel_q8;

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
    turn_active = active_bend = next_bend = finish_pending = 0;
    launch_pending = HAS_START_FINISH_STRIPE;
    clear_ticks = MARKER_CLEAR_TICKS;
    turn_ticks = straight_ticks = 0;
    launch_distance_twice = finish_distance_twice = turn_distance_twice = measured_speed_q8 = 0;
    debug_turn_distance_counts = debug_turn_min_counts = debug_ramped_pwm = 0;
    debug_speed_phase = PHASE_ACCELERATING;
    debug_turn_active = debug_launch_crossbar_ignored = 0;
    debug_turn_ticks = debug_straight_ticks = debug_turn_entries = debug_turn_exits =
        debug_finish_count = 0;
    debug_accelerating_ticks = debug_braking_ticks = 0;
    debug_left_encoder = debug_right_encoder = debug_measured_speed_q8 = debug_measured_accel_q8 =
        0;
}
void racer_control_step(void) {
    uint8_t sensors = read_sensors(), bar = activeLEDs(sensors) >= CROSSBAR_THRESHOLD &&
                                            (sensors & 0x03U) && (sensors & 0xc0U);
    uint8_t valid = sensors && !bar, integrate = 0;
    uint8_t marker_event = bar && !previous_bar && clear_ticks >= MARKER_CLEAR_TICKS;
    int16_t lenc = read_left_enc(), renc = read_right_enc();
    int16_t previous_base = base_pwm;
    int32_t travel_twice = (int32_t)abs16(lenc) + abs16(renc);
    int32_t previous_speed = measured_speed_q8;
    int16_t raw_error = last_error_q8, error = last_error_q8, limit, correction, common, left,
            right;
    int16_t request;
    int32_t p, d, candidate, raw_q8, leaked, applied_q8;
    // Encoder reads serve marker odometry and measurement only, never motor PI.
    measured_speed_q8 += (((int32_t)lenc + renc) * 128L - measured_speed_q8) / 4;
    debug_left_encoder = lenc;
    debug_right_encoder = renc;
    debug_measured_speed_q8 = (int16_t)measured_speed_q8;
    debug_measured_accel_q8 = (int16_t)(measured_speed_q8 - previous_speed);
    if (turn_active)
        turn_distance_twice += travel_twice;
    if (launch_pending)
        launch_distance_twice += travel_twice;
    if (finish_pending)
        finish_distance_twice += travel_twice;
    if (bar)
        clear_ticks = 0;
    else if (clear_ticks < MARKER_CLEAR_TICKS)
        clear_ticks++;
    if (marker_event) {
        uint8_t ignore = 0;
        if (debug_crossbar_count < UINT16_MAX)
            debug_crossbar_count++;
        if (launch_pending) {
            launch_pending = 0;
            if (launch_distance_twice <= 2L * MM_TO_COUNTS(100L)) {
                ignore = 1;
                debug_launch_crossbar_ignored = 1;
            }
        }
        if (!ignore && finish_pending) {
            if (finish_distance_twice < 2L * MM_TO_COUNTS(FINISH_MIN_MM)) {
                // An early broad pattern inside bend5 cannot be the finish.
                ignore = 1;
            } else if (finish_distance_twice <= 2L * MM_TO_COUNTS(FINISH_MAX_MM)) {
                ignore = 1;
                finish_pending = 0;
                if (debug_finish_count < UINT16_MAX)
                    debug_finish_count++;
            } else {
                // Missing finish stripe: do not consume the next real bend.
                finish_pending = 0;
            }
        }
        if (!ignore &&
            (!turn_active ||
             turn_distance_twice >= 2L * (turn_section_counts[active_bend] + MM_TO_COUNTS(150L)))) {
            // A new real marker cannot be lost just because the prior exit
            // never achieved a quiet enough line. Keep the slower effort.
            if (turn_active && debug_turn_exits < UINT16_MAX)
                debug_turn_exits++;
            turn_active = 1;
            turn_ticks = straight_ticks = 0;
            turn_distance_twice = 0;
            active_bend = next_bend;
            debug_turn_number = active_bend + 1U;
            next_bend = (next_bend + 1U) % 5U;
            if (debug_turn_entries < UINT16_MAX)
                debug_turn_entries++;
            if (HAS_START_FINISH_STRIPE && next_bend == 0) {
                finish_pending = 1;
                finish_distance_twice = 0;
            }
        }
    }
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
    if (turn_active) {
        if (turn_ticks < UINT16_MAX)
            turn_ticks++;
        if (valid && turn_ticks >= TURN_MIN_HOLD_TICKS &&
            turn_distance_twice >=
                2L * turn_section_counts[active_bend] * TURN_MIN_PROGRESS_PERCENT / 100L &&
            abs16(raw_error) <= STRAIGHT_ERROR_Q8 && abs16(filtered_d_q8) <= STRAIGHT_DELTA_Q8) {
            if (straight_ticks < STRAIGHT_CONFIRM_TICKS)
                straight_ticks++;
        } else
            straight_ticks = 0;
        if (straight_ticks >= STRAIGHT_CONFIRM_TICKS) {
            turn_active = 0;
            straight_ticks = 0;
            if (debug_turn_exits < UINT16_MAX)
                debug_turn_exits++;
        }
    }
    request = turn_active ? DECEL_PWM : ACCEL_PWM;
    if (!sensors && request > LOST_LINE_PWM)
        request = LOST_LINE_PWM;
    if (lost_ticks >= LOST_STOP_TICKS)
        request = 0;
    if (base_pwm < request)
        base_pwm = (int16_t)clip32(base_pwm + BASE_RISE_STEP, 0, request);
    else if (base_pwm > request)
        base_pwm = (int16_t)clip32(base_pwm - BASE_FALL_STEP, request, MOTOR_PWM_MAX);
    if (base_pwm < previous_base) {
        debug_speed_phase = PHASE_BRAKING;
        if (debug_braking_ticks < UINT32_MAX)
            debug_braking_ticks++;
    } else if (base_pwm > previous_base) {
        debug_speed_phase = PHASE_ACCELERATING;
        if (debug_accelerating_ticks < UINT32_MAX)
            debug_accelerating_ticks++;
    } else
        debug_speed_phase = turn_active ? PHASE_CORNER : PHASE_CRUISE;
    debug_turn_active = turn_active;
    debug_motion_state = turn_active;
    debug_turn_ticks = turn_ticks;
    debug_straight_ticks = straight_ticks;
    debug_turn_distance_counts = turn_distance_twice / 2;
    debug_turn_min_counts = turn_section_counts[active_bend] * TURN_MIN_PROGRESS_PERCENT / 100L;
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
    debug_ramped_pwm = base_pwm;
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
