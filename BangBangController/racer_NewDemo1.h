// racer.h
//
// Function prototypes for both the simulator and the Nano low level code. 
//
// This code is supplied as is and without warranty. While every care has been taking to ensure its accuracy
// and reliability, it may contain errors. Feel free to modify this code as you need for your project.
// (c) Gordon Wyeth 2026 - the author asserts moral rights to this code

#include <stdint.h>

// Function prototypes defining the interface
unsigned char read_sensors(void);
void set_speeds(int16_t l_speed, int16_t r_speed);
int8_t read_left_enc(void);
int8_t read_right_enc(void);

void toggle_led(void);
void led_on(void);
void led_off(void);