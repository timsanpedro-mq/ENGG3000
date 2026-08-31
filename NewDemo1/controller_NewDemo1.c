// POST-SPRINT 1 DEMO
// DEMO VERSION: 1.0
// DEMO FUNCTION: standard bang-bang controller 


#include "racer_NewDemo1.h"
#include <stdint.h>

// Use the stdint definitions of types so you know exactly how many bits of precision you have
// This will be important as the defaults for types like "int" will be different on the
// PC to the microcontroller

// These are the variables
#define ACCEL_PWM    70
#define DECEL_PWM    45 
#define CORR_PWM     53 

// TSP: the crossbar threshold is set to 5 as the crossbar activates a minimum of 5 LEDs
#define CROSSBAR_THRESHOLD 5

// TSP: currently only one state, implement states later 
// TSP: idea, use states for specific instructions for each turn? use the crossbar detection to transition states
#define RUN    1

int8_t state, next_state;   // The current and the next state

// This is called when the racer starts up
void racer_init(void) {
    state = next_state = RUN;
}

// This is the heart of your racer controller. It is called every 5ms (or whatever you set your timer to be)
// so it needs to run fast on the real robot. Don't put float calculations in here or too much 32 bit math
// involving multiplies and divides. If this takes more than 3ms to run your robot will become erratic.


// TSP: function to count how many LEDs are on at once. we use this to detect the turn crossbars and straights
uint8_t activeLEDs(uint8_t sensorvalue) {
    uint8_t countLED = 0;
    
    while(sensorvalue) {
        countLED += sensorvalue & 1;
        sensorvalue >>= 1;
    }
    return countLED;
}

void racer_control_step(void) {

    // Variables that will hold the final PWM values to send to the motors
    int16_t lw_pwm = 0;
    int16_t rw_pwm = 0;

    // Read all the sensors
    
    // TSP: encoder readings commented out for now, use later for better speed control and states
    // int16_t lenc = read_left_enc();
    // int16_t renc = read_right_enc();
    uint8_t sensors = read_sensors();

    // states are used so that you can switch control strategies for different parts of the track
    if (state == RUN) {
        
        // TSP: drive straight at our accel_pwm - no controller yet.
        // TSP: idea, change this line to an equation with proportional, integral or derivative control
        lw_pwm = rw_pwm = ACCEL_PWM;
        
        // TSP: if more than 5 LEDs are detected, assume we are at a turn crossbar and slow down the PWM
        if (activeLEDs(sensors) >= CROSSBAR_THRESHOLD) lw_pwm = rw_pwm = DECEL_PWM;
        
        // Check the sensor values to keep racer on the track. 
        
        // 0xE0 is 11100000, so sensors & 0xE0 means one of the three rightmost sensors is on
        // TSP: slow the right motor to turn right
        else if (sensors & 0xE0) rw_pwm -= CORR_PWM;
        
        // 0x07 is 00000111, so sensors & 0x07 means one of the three leftmost sensors is on
        // TSP: slow the left motor to turn left
        else if (sensors & 0x07) lw_pwm -= CORR_PWM;
    }
    
    // Final step is to send the computed PWM to the motors
    set_speeds(lw_pwm, rw_pwm);
}
