// sim_link.c
#include "racer.h"

// Internal function pointers that point back to Python's ctypes hooks
unsigned char (*get_sensors_fp)(void) = 0;
void (*set_speeds_fp)(int16_t, int16_t) = 0;
int8_t (*read_left_enc_fp)(void) = 0;
int8_t (*read_right_enc_fp)(void) = 0;

// The initialization hook called by drawing.py at boot
void init_simulator_links(unsigned char (*sens_cb)(void), void (*speed_cb)(int16_t, int16_t), int8_t (*lenc_cb)(void), int8_t (*renc_cb)(void)) {
    get_sensors_fp = sens_cb;
    set_speeds_fp = speed_cb;
    read_left_enc_fp = lenc_cb;
    read_right_enc_fp = renc_cb;
}

// Wrapper implementations of the HAL functions declared in low_level.h
unsigned char read_sensors(void) {
    if (get_sensors_fp) return get_sensors_fp();
    return 0;
}

void set_speeds(int16_t l_speed, int16_t r_speed) {
    if (set_speeds_fp) set_speeds_fp(l_speed, r_speed);
}

int8_t read_left_enc(void) {
    if (read_left_enc_fp) return read_left_enc_fp();
}

int8_t read_right_enc(void) {
    if (read_right_enc_fp) return read_right_enc_fp();
}

void toggle_led(void){};
void led_on(void){};
void led_off(void){};