// DEMO VERSION: 7 - translational PI with straight PD and turn PI
// This demo has track trajectories implemented, but no accel and decel on straights
// Straight steering follows a PD line follower, while turn steering follows the PI rotational control

#include "racer.h"
#include <stdint.h>

// Speed targets: encoder counts per 5 ms step
#define STRAIGHT_SPEED 60
#define R100_SPEED     9
#define R150_SPEED     9
#define R200_SPEED     9

// Confirm these values against the physical racer.
#define ENCODER_COUNTS_PER_REV 360L

// Simulator: 30 mm diameter (racer.py uses wheel_radius = 0.015 m).
// Physical car: change this to 15L if its measured wheel diameter is 15 mm.
#define WHEEL_DIAMETER_MM       30L

// For a 90-degree arc: counts = R*counts_per_rev/(2*wheel_diameter).
#define QUARTER_ARC_COUNTS(radius_mm) \
    (((int32_t)(radius_mm) * ENCODER_COUNTS_PER_REV) / \
     (2L * WHEEL_DIAMETER_MM))

#define R100_COUNTS QUARTER_ARC_COUNTS(100L) // 600 counts
#define R150_COUNTS QUARTER_ARC_COUNTS(150L) // 900 counts
#define R200_COUNTS QUARTER_ARC_COUNTS(200L) // 1200 counts

#define NUM_TURNS          6U
#define MAX_RADII_PER_TURN 4U

typedef struct {
    int16_t speed;
    int32_t distance_counts;
} RadiusStage;

typedef struct {
    uint8_t number_of_radii;
    RadiusStage radius[MAX_RADII_PER_TURN];
} TurnSection;

// Turn 0: Straight Speed
// Turn 1: R100+R100; Turn 2: R200; Turn 3: R200;
// Turn 4: R150+R150; Turn 5: (2*R150)+R100+R150.
static const TurnSection turn_plan[NUM_TURNS] = {
    {1U, {STRAIGHT_SPEED, 5L}},
    {2U, {{R100_SPEED, R100_COUNTS}, {R100_SPEED, R100_COUNTS}}},
    {1U, {{R200_SPEED, R200_COUNTS}}},
    {1U, {{R200_SPEED, R200_COUNTS}}},
    {2U, {{R150_SPEED, R150_COUNTS}, {R150_SPEED, R150_COUNTS}}},
    {3U, {{R150_SPEED, 2L * R150_COUNTS}, {R100_SPEED, R100_COUNTS}, {R150_SPEED, R150_COUNTS}}}
};

#define STEER_GAIN 1

// NewDemo3 PD law. KD includes the fixed 5 ms sample interval.
#define STRAIGHT_KP 6
#define STRAIGHT_KD 3
#define STRAIGHT_PD_MAX 40
#define STRAIGHT_STEERING_SIGN 1

static int16_t pd_prev_error = 0;
static uint8_t pd_history_valid = 0;


// Translational PI
#define KTRP_NUM       42
#define KTRI_NUM        2
#define TR_SHIFT        4
#define TR_INTEG_MAX  200
#define TR_EFFORT_MAX  80

// Rotational PI
#define KROTP_NUM      24
#define KROTI_NUM       3
#define ROT_SHIFT       4
#define ROT_INTEG_MAX 200
#define ROT_EFFORT_MAX 60

// Detection settings
#define CB_THRESHOLD              5U
#define MIN_CROSSBAR_TICKS       10U

// Remain at the final turn speed briefly after completing the planned turn
// distance. 40 control ticks x 5 ms = 200 ms.
#define TURN_EXIT_DELAY_TICKS    10U

#define STATE_STRAIGHT 0U
#define STATE_TURN     1U

static uint8_t motion_state = STATE_STRAIGHT;
static uint8_t next_turn = 0;
static uint8_t active_turn = 0;
static uint8_t active_radius = 0;
static uint8_t turn_distance_complete = 0;
static int32_t radius_distance_counts = 0;

static int16_t i_straight = 0;
static int16_t i_turn = 0;
static int16_t last_line_err = 0;

static uint8_t crossbar_passed = 0;
static uint16_t crossbar_ticks_passed = 0;
static uint16_t turn_exit_ticks = 0;

static const int8_t sensor_weight[8] = {
    -7, -5, -3, -1, 1, 3, 5, 7
};

// Debugger-visible values
volatile uint8_t debug_motion_state = STATE_STRAIGHT;
volatile uint8_t debug_turn_number = 0;
volatile uint8_t debug_radius_number = 0;
volatile int16_t debug_speed_target = STRAIGHT_SPEED;
volatile int32_t debug_radius_counts = 0;
volatile int32_t debug_radius_target = 0;
volatile int16_t debug_line_error = 0;
volatile int16_t debug_pd_derivative = 0;
volatile int16_t debug_steering_effort = 0;
volatile uint8_t debug_straight_pd_active = 0;

static int16_t abs16(int16_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t count_sensors(uint8_t sensor_value)
{
    uint8_t count = 0;
    while (sensor_value != 0U) {
        count += sensor_value & 1U;
        sensor_value >>= 1;
    }
    return count;
}

static int16_t read_line_err(uint8_t sensors)
{
    int16_t error = 0;
    uint8_t count = 0;
    uint8_t i;

    for (i = 0; i < 8U; i++) {
        if ((sensors & (1U << i)) != 0U) {
            error += sensor_weight[i];
            count++;
        }
    }

    if (count == 0U) {
        return last_line_err;
    }

    error /= count;
    last_line_err = error;
    return error;
}

// Average wheel movement equals chassis-centre distance for differential drive.
static int16_t average_encoder_counts(int16_t lenc, int16_t renc)
{
    return (abs16(lenc) + abs16(renc)) / 2;
}

static void enter_turn(void)
{
    motion_state = STATE_TURN;
    active_turn = next_turn;

    next_turn++;
    if (next_turn >= NUM_TURNS) {
        next_turn = 0;
    }

    active_radius = 0;
    radius_distance_counts = 0;
    turn_distance_complete = 0;
    turn_exit_ticks = 0;
    i_straight = 0;
    i_turn = 0;
    pd_history_valid = 0;
}

static void enter_straight(void)
{
    motion_state = STATE_STRAIGHT;
    radius_distance_counts = 0;
    turn_distance_complete = 0;
    turn_exit_ticks = 0;
    i_straight = 0;
    i_turn = 0;
    pd_history_valid = 0;
}

static void update_turn_distance(int16_t lenc, int16_t renc)
{
    int32_t required_counts;

    if (turn_distance_complete != 0U) {
        return;
    }

    radius_distance_counts += average_encoder_counts(lenc, renc);
    required_counts =
        turn_plan[active_turn].radius[active_radius].distance_counts;

    if (radius_distance_counts >= required_counts) {
        // Carry encoder overshoot into the following radius.
        radius_distance_counts -= required_counts;
        active_radius++;

        if (active_radius >= turn_plan[active_turn].number_of_radii) {
            active_radius = turn_plan[active_turn].number_of_radii - 1U;
            turn_distance_complete = 1U;
            turn_exit_ticks = 0;
        }
    }
}

void racer_init(void)
{
    motion_state = STATE_STRAIGHT;
    next_turn = 0;
    active_turn = 0;
    active_radius = 0;
    turn_distance_complete = 0;
    radius_distance_counts = 0;

    i_straight = 0;
    i_turn = 0;
    pd_history_valid = 0;
    last_line_err = 0;
    pd_prev_error = 0;
    debug_line_error = 0;
    debug_pd_derivative = 0;
    debug_steering_effort = 0;
    debug_straight_pd_active = 0;
    crossbar_passed = 0;
    crossbar_ticks_passed = 0;
    turn_exit_ticks = 0;

    debug_motion_state = motion_state;
    debug_turn_number = 0;
    debug_radius_number = 0;
    debug_speed_target = STRAIGHT_SPEED;
    debug_radius_counts = 0;
    debug_radius_target = 0;
}

void racer_control_step(void)
{
    int16_t lenc = read_left_enc();
    int16_t renc = read_right_enc();
    uint8_t sensors = read_sensors();

    uint8_t on_crossbar;
    uint8_t new_crossbar;
    int16_t V_SET;
    int16_t line_err;
    int16_t wL_set;
    int16_t wR_set;
    int16_t errorL;
    int16_t errorR;
    int16_t straight_error;
    int16_t turn_error;
    int16_t trans_out;
    int16_t rot_out;
    int16_t lw_pwm;
    int16_t rw_pwm;
    int16_t derivative = 0;
    int16_t steering_limit;
    int32_t pd_correction;
    uint8_t line_valid;

    // Crossbar rising edge selects the next turn section.
    if (crossbar_ticks_passed < UINT16_MAX) {
        crossbar_ticks_passed++;
    }

    on_crossbar = (count_sensors(sensors) >= CB_THRESHOLD);
    new_crossbar = on_crossbar && !crossbar_passed &&
                   (crossbar_ticks_passed > MIN_CROSSBAR_TICKS);

    if (new_crossbar && (motion_state == STATE_STRAIGHT)) {
        enter_turn();
        crossbar_ticks_passed = 0;
    }
    crossbar_passed = on_crossbar;

    // Distance advances through the planned radii. After the full expected
    // turn distance, keep the final turn speed for a short settling delay and
    // then switch directly to the straight state.
    if (motion_state == STATE_TURN) {
        update_turn_distance(lenc, renc);

        if (turn_distance_complete != 0U) {
            if (turn_exit_ticks < TURN_EXIT_DELAY_TICKS) {
                turn_exit_ticks++;
            }

            if (turn_exit_ticks >= TURN_EXIT_DELAY_TICKS) {
                enter_straight();
            }
        }
    }
    else {
        turn_exit_ticks = 0;
    }

    // Constant speed for the current straight or radius.
    if (motion_state == STATE_TURN) {
        V_SET = turn_plan[active_turn].radius[active_radius].speed;
    }
    else {
        V_SET = STRAIGHT_SPEED;
    }

    // Left/right wheel targets from forward demand plus line correction.
    // Crossbar centroids are not a reliable lateral position measurement.
    line_valid = (sensors != 0U) && !on_crossbar;
    line_err = on_crossbar ? last_line_err : read_line_err(sensors);
    wL_set = V_SET + STEER_GAIN * line_err;
    wR_set = V_SET - STEER_GAIN * line_err;

    errorL = wL_set - lenc;
    errorR = wR_set - renc;

    // Sum isolates common translation; difference isolates rotation.
    straight_error = errorL + errorR;
    turn_error = errorL - errorR;

    // Translational PI
    i_straight += straight_error;
    if (i_straight > TR_INTEG_MAX) i_straight = TR_INTEG_MAX;
    if (i_straight < -TR_INTEG_MAX) i_straight = -TR_INTEG_MAX;

    trans_out =
        (KTRP_NUM * straight_error + KTRI_NUM * i_straight) >> TR_SHIFT;
    if (trans_out > TR_EFFORT_MAX) trans_out = TR_EFFORT_MAX;
    if (trans_out < 0) trans_out = 0;

    if (motion_state == STATE_STRAIGHT) {
        // One steering controller at a time: direct PD on straights.
        // Forward effort still comes exclusively from translational PI.
        if (line_valid) {
            if (pd_history_valid) derivative = line_err - pd_prev_error;
            pd_prev_error = line_err;
            pd_history_valid = 1;
        } else {
            // No D kick on line reacquisition or when leaving a crossbar.
            // Hold the last proportional correction while the line is absent.
            pd_history_valid = 0;
        }

        pd_correction = STRAIGHT_STEERING_SIGN *
            ((int32_t)STRAIGHT_KP * line_err +
             (int32_t)STRAIGHT_KD * derivative);
        steering_limit = STRAIGHT_PD_MAX;
        // Keep both straight wheel commands forward and <=100, as in
        // NewDemo3, without independently clipping away the mean effort.
        if (steering_limit > trans_out) steering_limit = trans_out;
        if (steering_limit > 100 - trans_out) steering_limit = 100 - trans_out;
        if (steering_limit < 0) steering_limit = 0;
        if (pd_correction > steering_limit) pd_correction = steering_limit;
        if (pd_correction < -steering_limit) pd_correction = -steering_limit;
        rot_out = (int16_t)pd_correction;
        i_turn = 0; // No hidden rotational-integrator accumulation on straights.
    } else {
        // Original rotational PI for all turn stages and their exit delay.
        pd_history_valid = 0;
        i_turn += turn_error;
        if (i_turn > ROT_INTEG_MAX) i_turn = ROT_INTEG_MAX;
        if (i_turn < -ROT_INTEG_MAX) i_turn = -ROT_INTEG_MAX;

        rot_out =
            (KROTP_NUM * turn_error + KROTI_NUM * i_turn) >> ROT_SHIFT;
        if (rot_out > ROT_EFFORT_MAX) rot_out = ROT_EFFORT_MAX;
        if (rot_out < -ROT_EFFORT_MAX) rot_out = -ROT_EFFORT_MAX;
    }

    // Recombine common and differential efforts into motor commands.
    lw_pwm = trans_out - rot_out;
    rw_pwm = trans_out + rot_out;

    debug_line_error = line_err;
    debug_pd_derivative = derivative;
    debug_steering_effort = rot_out;
    debug_straight_pd_active = (motion_state == STATE_STRAIGHT);
    debug_motion_state = motion_state;
    debug_turn_number = active_turn + 1U;
    debug_radius_number = active_radius + 1U;
    debug_speed_target = V_SET;
    debug_radius_counts = radius_distance_counts;
    debug_radius_target = (motion_state == STATE_TURN)
        ? turn_plan[active_turn].radius[active_radius].distance_counts
        : 0;

    set_speeds(lw_pwm, rw_pwm);
}