// POST-SPRINT 1 DEMO
// DEMO VERSION: 3.0
// DEMO FUNCTION: standard proportional-derivative controller 
#include "racer_NewDemo3.h"
#include <stdint.h>

#define ACCEL_PWM   55
#define DECEL_PWM   45 

// TSP: proportional constant
#define KP          6
// TSP: derivative constant. dt = 5ms is fixed, so it's folded into this gain
// rather than divided out at runtime. Start small (1-3) and tune up ?
// too high and you'll amplify sensor noise instead of damping oscillation.
#define KD          2

#define CROSSBAR_THRESHOLD 5

#define RUN    1
int8_t state, next_state;

// NEW: persists between calls so we can compute error delta
static int16_t prevError = 0;

void racer_init(void) {
    state = next_state = RUN;
    prevError = 0;   // reset on init too
}

uint8_t activeLEDs(uint8_t sensorvalue) {
    uint8_t countLED = 0;
    while(sensorvalue) {
        countLED += sensorvalue & 1;
        sensorvalue >>= 1;
    }
    return countLED;
}

int16_t getLineError(uint8_t sensors)
{
    int16_t weightedSum = 0;
    uint8_t active = 0;
    const int8_t position[8] =
        {-7, -5, -3, -1, 1, 3, 5, 7};
    for (uint8_t i = 0; i < 8; i++)
    {
        if (sensors & (1 << i))
        {
            weightedSum += position[i];
            active++;
        }
    }
    if (active == 0)
        return 0;
    return weightedSum / active;
}

void racer_control_step(void) {
    int16_t lw_pwm = 0;
    int16_t rw_pwm = 0;

    uint8_t sensors = read_sensors();

    if (state == RUN) {

        if (activeLEDs(sensors) >= CROSSBAR_THRESHOLD) {
            lw_pwm = rw_pwm = DECEL_PWM;
            // TSP: reset prevError so we don't get a derivative spike
            // from a stale error value when we re-enter line-following
            prevError = 0;
        }
        else {
            int16_t error = getLineError(sensors);

            // NEW: derivative term = rate of change of error
            int16_t derivative = error - prevError;
            prevError = error;

            // Use int32_t intermediate to avoid overflow before clamping
            int32_t correction = (int32_t)KP * error + (int32_t)KD * derivative;

            lw_pwm = ACCEL_PWM - correction;
            rw_pwm = ACCEL_PWM + correction;
        }
    }

    if (lw_pwm > 100) lw_pwm = 100;
    if (lw_pwm < 0)   lw_pwm = 0;
    if (rw_pwm > 100) rw_pwm = 100;
    if (rw_pwm < 0)   rw_pwm = 0;

    set_speeds(lw_pwm, rw_pwm);
}