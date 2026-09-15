// DEMO VERSION: 5.0
// DEMO FUNCTION: Double PI controller w/ STRAIGHT and TURN states
//
// Operation:
// 1. Racer starts at constant straight speed.
// 2. Crossbar changes the state to TURN.
// 3. Racer uses a slower constant speed through the turn.
// 4. Encoder agreement detects exit from the turn.
// 5. Racer returns to constant straight speed.
// 6. The sequence repeats.

#include "racer.h"
#include <stdint.h>


// Set speed targets per 5 ms controller step
#define STRAIGHT_SPEED    50
#define TURN_SPEED        10

// Effect of line position on differential wheel-speed targets
#define STEER_GAIN         1

/// Translational PI Controller Variables
#define KTRP_NUM          42
#define KTRI_NUM           2
#define TR_SHIFT           4

#define TR_INTEG_MAX     200
#define TR_EFFORT_MAX     80

// Rotational PI Controller Variables
#define KROTP_NUM         24
#define KROTI_NUM          3
#define ROT_SHIFT          4

#define ROT_INTEG_MAX    200
#define ROT_EFFORT_MAX    60

// Number of active sensors required to detect a crossbar
#define CB_THRESHOLD 5

// Min delay between crossbar detections
// 40 ticks × 5 ms = 200 ms
#define MIN_CROSSBAR_TICKS 40

// Maximum difference between left and right encoder increments
#define STRAIGHT_ENC_TOL 2

// Min time to detect a straight
// 40 ticks × 5 ms = 200 ms
#define STRAIGHT_CONFIRM_TICKS 40

// Stops a stationary racer being classified as straight
#define MIN_FORWARD_COUNTS 2

// Min time for turn state to transition back into a straight state
// 40 ticks × 5 ms = 200 ms
#define MIN_TURN_TICKS 40

// Racer States
#define STATE_STRAIGHT 0
#define STATE_TURN     1

static uint8_t motion_state = STATE_STRAIGHT;

// PI controller variables
static int16_t i_straight = 0;
static int16_t i_turn = 0;

// Sensor line correction variables/weighted array
static int16_t last_line_err = 0;

static const int8_t sensor_weight[8] = {
    -7, -5, -3, -1, 1, 3, 5, 7
};

// Crossbar detection variables
static uint8_t crossbar_passed = 0;
static uint16_t crossbar_ticks_passed = 0;

// Turn and Straight variables
static uint16_t turn_ticks = 0;
static uint16_t straight_ticks = 0;

// Absolute Value function to convert signed integers into unsigned
static int16_t abs16(int16_t value) {
    return (value < 0) ? -value : value;
}


// Counts how many sensors currently read white
// White = 1; Black = 0
static uint8_t count_sensors(uint8_t sensor_value) {
    uint8_t count = 0;

    while (sensor_value != 0) {
        count += sensor_value & 1U;
        sensor_value >>= 1;
    }

    return count;
}

// Calculates the line-position error
// Negative error: line is toward one side
// Positive error: line is toward the other side
static int16_t read_line_err(uint8_t sensors) {
    int16_t error = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (sensors & (1U << i)) {
            error += sensor_weight[i];
            count++;
        }
    }

    // Return last line error if the sensors detect nothing
    if (count == 0) {
        return last_line_err;
    }

    error /= count;
    last_line_err = error;

    return error;
}

// Straight detection function using encoders
static uint8_t detect_straight(int16_t lenc, int16_t renc) {
    int16_t left_counts = abs16(lenc);
    int16_t right_counts = abs16(renc);

    int16_t forward_counts =
        (left_counts + right_counts) / 2;

    int16_t encoder_difference =
        abs16(left_counts - right_counts);

    if ((forward_counts >= MIN_FORWARD_COUNTS) &&
        (encoder_difference <= STRAIGHT_ENC_TOL)) {

        if (straight_ticks < STRAIGHT_CONFIRM_TICKS) {
            straight_ticks++;
        }
    }
    else {
        straight_ticks = 0;
    }

    return (straight_ticks >= STRAIGHT_CONFIRM_TICKS);
}

// Straight to Turn variables reset
static void enter_turn(void) {
    motion_state = STATE_TURN;

    turn_ticks = 0;
    straight_ticks = 0;

    // Reset PI Variables as we're instantaneously changing speeds from straight to turn
    i_straight = 0;
    i_turn = 0;
}

// Turn to Straight variables reset
static void enter_straight(void) {
    motion_state = STATE_STRAIGHT;

    turn_ticks = 0;
    straight_ticks = 0;

    // Reset PI Variables as we're instantaneously changing speeds from straight to turn
    i_straight = 0;
    i_turn = 0;
}

// Start Racer 
void racer_init(void) {
    // Racer starts with the initial constant-speed dash
    motion_state = STATE_STRAIGHT;

    // Initialise all variables at 0
    i_straight = 0;
    i_turn = 0;

    last_line_err = 0;

    crossbar_passed = 0;
    crossbar_ticks_passed = 0;

    turn_ticks = 0;
    straight_ticks = 0;
}

// Main Racer Controller called every 5ms
void racer_control_step(void) {
    // Read Sensors
    int16_t lenc = read_left_enc();
    int16_t renc = read_right_enc();
    uint8_t sensors = read_sensors();
    
   // Detect Crossbar
    if (crossbar_ticks_passed < UINT16_MAX) {
        crossbar_ticks_passed++;
    }

    uint8_t on_crossbar = (count_sensors(sensors) >= CB_THRESHOLD);

    // Use rising edge of the crossbar detection
    uint8_t new_crossbar = (on_crossbar && !crossbar_passed && (crossbar_ticks_passed > MIN_CROSSBAR_TICKS));

    if (new_crossbar && motion_state == STATE_STRAIGHT) {
        enter_turn();
        crossbar_ticks_passed = 0;
    }

    crossbar_passed = on_crossbar;

    // Detect exit turn into straight
    if (motion_state == STATE_TURN) {

        if (turn_ticks < UINT16_MAX) {
            turn_ticks++;
        }

        // Wait for min time to remain on turn; prevents quick changes back to straight
        if (turn_ticks >= MIN_TURN_TICKS) {

            uint8_t straight_detected = detect_straight(lenc, renc);

            if (straight_detected) {
                enter_straight();
            }
        }
    }
    else {
        turn_ticks = 0;
        straight_ticks = 0;
    }

    // set V_SET to either turn or straight speed
    int16_t V_SET;

    if (motion_state == STATE_TURN) {
        V_SET = TURN_SPEED;
    }
    else {
        V_SET = STRAIGHT_SPEED;
    }

    // Calculate line-following errors
    int16_t line_err = read_line_err(sensors);

    // Calculate into w_set for PI Control
    int16_t wL_set =
        V_SET + STEER_GAIN * line_err;

    int16_t wR_set =
        V_SET - STEER_GAIN * line_err;


    // Calculate encoder speed errors
    int16_t errorL = wL_set - lenc;
    int16_t errorR = wR_set - renc;

    int16_t straight_error =
        errorL + errorR;

    int16_t turn_error =
        errorL - errorR;

    // Translational PI Controller
    i_straight += straight_error;

    if (i_straight > TR_INTEG_MAX) {
        i_straight = TR_INTEG_MAX;
    }

    if (i_straight < -TR_INTEG_MAX) {
        i_straight = -TR_INTEG_MAX;
    }

    int16_t trans_out =
        (KTRP_NUM * straight_error +
         KTRI_NUM * i_straight) >> TR_SHIFT;

    if (trans_out > TR_EFFORT_MAX) {
        trans_out = TR_EFFORT_MAX;
    }

    if (trans_out < 0) {
        trans_out = 0;
    }

    // Rotational PI Controller
    i_turn += turn_error;

    if (i_turn > ROT_INTEG_MAX) {
        i_turn = ROT_INTEG_MAX;
    }

    if (i_turn < -ROT_INTEG_MAX) {
        i_turn = -ROT_INTEG_MAX;
    }

    int16_t rot_out =
        (KROTP_NUM * turn_error +
         KROTI_NUM * i_turn) >> ROT_SHIFT;

    if (rot_out > ROT_EFFORT_MAX) {
        rot_out = ROT_EFFORT_MAX;
    }

    if (rot_out < -ROT_EFFORT_MAX) {
        rot_out = -ROT_EFFORT_MAX;
    }

    // Set Motor Commands as output of trans and rot controllers
    int16_t lw_pwm = trans_out + rot_out;
    int16_t rw_pwm = trans_out - rot_out;

    set_speeds(lw_pwm, rw_pwm);
}