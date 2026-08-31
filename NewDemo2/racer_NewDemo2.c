
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#define F_CPU 16000000UL 
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include "racer_NewDemo2.h"
#include "controller_NewDemo2.h"


// Hardware Pin Masks
#define QTR_MASK_C 0x3F  // PC0 to PC5 (Sensors 0-5)
#define QTR_MASK_B 0x0C  // PB2 to PB3 (Sensors 6-7)

// Sensor threshold: adjust this depending on ambient light and ride height!
// 2000us is a standard starting point for Pololu RC sensors.
#define THRESHOLD_US 200

// Internal prototypes
void motor_init(void);
void adc_init(void);
uint16_t adc_read(uint8_t channel);
void qtr_leds_init(void); 
void qtr_leds_on(void);
void qtr_leds_off(void);

// The encoder tick counts - these can overflow
volatile uint8_t motor1_ticks = 0;
volatile uint8_t motor2_ticks = 0;

// Store the previous state (2 bits each)
static uint8_t m1_old_state = 0;
static uint8_t m2_old_state = 0;
uint8_t last_m1 = 0;
uint8_t last_m2 = 0;

// Constants to use USB port to print to PC
// Define clock speed and target baud rate explicitly
#define BAUD_RATE 115200
// Use 8UL for Double Speed Mode math
#define UBRR_VAL ((F_CPU / (8UL * BAUD_RATE)) - 1)


// Timer 1 Compare Match A Interrupt Service Routine (ISR)
// This fires exactly 500 times per second and calls the racer_control_step in 
// controller.c
ISR(TIMER1_COMPA_vect) {
    racer_control_step();
}


// The 16-state Quadrature Lookup Table for encoder mapping
// Maps [Old State (2 bits) | New State (2 bits)] to direction (+1, -1, or 0)
static const int8_t ENC_LUT[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

// Set up the encoders as per the schematic
void encoder_init(void) {
    // Set PD2, PD3, PB0, PB1 as inputs
    DDRD &= ~((1 << DDD2) | (1 << DDD3)); 
    DDRB &= ~((1 << DDB0) | (1 << DDB1));

    // Enable internal pull-up resistors to keep the signals perfectly square
    PORTD |= (1 << PORTD2) | (1 << PORTD3);
    PORTB |= (1 << PORTB0) | (1 << PORTB1);

    // Enable Pin Change Interrupt Banks for Port B and Port D
    PCICR |= (1 << PCIE0) | (1 << PCIE2);

    // Unmask the specific pins
    PCMSK2 |= (1 << PCINT18) | (1 << PCINT19); // PD2, PD3
    PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);   // PB0, PB1

    // Seed the initial states so the first tick isn't miscalculated
    m1_old_state = (PIND & 0x0C) >> 2;
    m2_old_state = (PINB & 0x03);
}

// ---------------------------------------------------------
// HIGH-SPEED ISR: Encoder on Motor 1 (PD2, PD3)
// This is called every encoder click so must be super fast
// ---------------------------------------------------------
ISR(PCINT2_vect) {
    // 1. Read Port D and mask out bits 2 and 3, then shift them down to bits 0 and 1
    uint8_t new_state = (PIND & 0x0C) >> 2; 

    // 2. Shift the old state up 2 bits and OR it with the new state to build the index
    uint8_t index = (m1_old_state << 2) | new_state;

    // 3. Apply the table delta and save the state
    motor1_ticks += ENC_LUT[index];
    m1_old_state = new_state;
}

// ---------------------------------------------------------
// HIGH-SPEED ISR: Motor 2 (PB0, PB1)
// This is called every encoder click so must be super fast
// ---------------------------------------------------------
ISR(PCINT0_vect) {
    // 1. Read Port B and mask out bits 0 and 1 (No bit-shifting required!)
    uint8_t new_state = (PINB & 0x03); 

    // 2. Build the index
    uint8_t index = (m2_old_state << 2) | new_state;

    // 3. Apply the table delta and save the state
    motor2_ticks += ENC_LUT[index];
    m2_old_state = new_state;
}

// Function to turn rolling encoder counts into successive reads. Assumes
// calls are made before the total count reaches 127 - which is fine so long
// as encoder are ready every 5ms
int8_t read_left_enc(void) {
    // Keep track of what the counter was last loop

    int8_t delta_m1;
    
    uint8_t current_m1 = motor1_ticks;

    delta_m1 = (int8_t)(current_m1 - last_m1);

    // Save current state for the next read
    last_m1 = current_m1;
    
    return(delta_m1);

}

int8_t read_right_enc(void) {
    // Keep track of what the counter was last loop

    int8_t delta_m2;
    
    uint8_t current_m2 = motor2_ticks;

    delta_m2 = (int8_t)(last_m2 - current_m2);

    // Save current state for the next read
    last_m2 = current_m2;
    
    return(delta_m2);
}

// Set up the interrupt to trigger every 5ms to call the control loop
void init_timer1_200hz(void) {
    // 1. Set CTC mode (WGM12 = 1, rest of WGM bits = 0)
    TCCR1A = 0; 
    TCCR1B = (1 << WGM12);
    
    // 2. Set the compare match value calculated above
    OCR1A = 1249; //499
    
    // 3. Enable the Timer 1 Compare Match A interrupt
    TIMSK1 |= (1 << OCIE1A);
    
    // 4. Set the prescaler to 64 and start the timer
    // CS11 = 1, CS10 = 1
    TCCR1B |= (1 << CS11) | (1 << CS10);
}

// A character transmission function that connects into the standard C 
// printf() function for debugging
static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream); // Add carriage return
    }
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = c;
    return 0;
}

// Create a stream binding for printf()
static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

// Set up the UART hardware so printf() can send data through the USB port
void debug_init(void) {
    UBRR0H = (unsigned char)(UBRR_VAL >> 8);
    UBRR0L = (unsigned char)UBRR_VAL; 
    
    // Enable Double Speed Mode for accurate 115200 timing!
    UCSR0A |= (1 << U2X0); 
    
    // Enable transmitter
    UCSR0B = (1 << TXEN0);
    
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    stdout = &uart_output;
}

// Turn on the user controlled LED
void led_on(void) {
    PORTB |= (1 << PORTB5); 
}

// Turn off the user controlled LED
void led_off(void) {
    PORTB &= ~(1 << PORTB5); 
}

// Toggle the user controlled LED
void toggle_led(void) {
    PORTB ^= (1 << PORTB5); 
}

// Initialise the PWM systems to vary voltage on motors
void motor_init(void) {
    // 1. Set D4, D5, D6, D7 as outputs
    DDRD |= (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7); 
    
    // 2. Configure Timer 0 (Pins D5, D6)
    // Phase Correct PWM (WGM00=1), Non-Inverting (COM0A1=1, COM0B1=1)
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
    // Prescaler 8 -> 3.92 kHz Carrier
    TCCR0B = (1 << CS01); 
    
    // Initialize to Full Brake (0 Speed)
    PORTD |= (1 << PORTD4) | (1 << PORTD7); // IN1 (D4, D7) HIGH
    OCR0B = 255; // Left IN2 (D5) HIGH 
    OCR0A = 255; // Right IN2 (D6) HIGH
}

// Sets up the PWM values. 
// ###!!! BEWARE ### !!! Because of the limitations of the MKRVRS motor
// driver the motors drive linearly in the forward direction and have a large
// stop band in reverse. Wire up your motors accordingly.
void set_speeds(int16_t left, int16_t right) {
    // Bound inputs
    if (left > 255) left = 255;
    if (left < -255) left = -255;
    if (right > 255) right = 255;
    if (right < -255) right = -255;

    // --- LEFT MOTOR (D4 = IN1, D5 = IN2) ---
    if (left >= 0) {
        // FORWARD: Slow Decay (IN1 High, IN2 Inverted PWM)
        PORTD |= (1 << PORTD4); 
        OCR0B = 255 - left;     
    } else {
        // REVERSE: Fast Decay (IN1 Low, IN2 Standard PWM)
        PORTD &= ~(1 << PORTD4); 
        OCR0B = -left;           
    }

    // --- RIGHT MOTOR (D7 = IN1, D6 = IN2) ---
    if (right >= 0) {
        // FORWARD: Slow Decay (IN1 High, IN2 Inverted PWM)
        PORTD |= (1 << PORTD7);  
        OCR0A = 255 - right;     
    } else {
        // REVERSE: Fast Decay (IN1 Low, IN2 Standard PWM)
        PORTD &= ~(1 << PORTD7); 
        OCR0A = -right;          
    }
}

// Reads the sensors and returns them as 8 bits. Note that this is blocking
// code that waits around for the sensors and will slow down your main 5ms loop.
// Use once at the top of the loop only.
unsigned char read_sensors(void) {
    
    unsigned char s;
    
    qtr_leds_on();
    
    // 1. Set pins as OUTPUT
    DDRC |= QTR_MASK_C;
    DDRB |= QTR_MASK_B;
    
    // 2. Drive pins HIGH to charge the capacitors
    PORTC |= QTR_MASK_C;
    PORTB |= QTR_MASK_B;
    
    // 3. Wait exactly 15 microseconds for a full charge
    _delay_us(15);
    
    // 4. Set pins as INPUT
    DDRC &= ~QTR_MASK_C;
    DDRB &= ~QTR_MASK_B;
    
    // 5. CRITICAL: Disable internal pull-up resistors
    PORTC &= ~QTR_MASK_C;
    PORTB &= ~QTR_MASK_B;

    // 6. Wait for the fixed threshold time
    _delay_us(THRESHOLD_US);

    // 7. Take the snapshot
    uint8_t current_c = PINC & QTR_MASK_C; // Captures bits 0-5
    uint8_t current_b = PINB & QTR_MASK_B; // Captures bits 2-3

    qtr_leds_off();
    
    // 8. Pack into a single 8-bit byte
    // Shift Port B's bits 2 and 3 up by 4 positions so they become bits 6 and 7
    s = ~(current_c | (current_b << 4));
    return s;
}

// Set up the ADC. We need this to check the battery voltage and also to read
// stop / go pushbutton.
void adc_init(void) {
    // 1. Set Reference to AVCC (5V). 
    // We leave the MUX bits at 0 for now; we will set them dynamically.
    ADMUX = (1 << REFS0);

    // 2. Enable ADC and set Prescaler to 128 (125kHz clock)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Channel 6 for the Button, or 7 for the Battery
uint16_t adc_read(uint8_t channel) {
    // 1. Clear the bottom 4 bits (MUX[3:0]) without touching the reference bits
    ADMUX &= 0xF0;
    
    // 2. Insert the desired channel (0 to 7)
    ADMUX |= (channel & 0x0F);
    
    // 3. Start the conversion
    ADCSRA |= (1 << ADSC);
    
    // 4. Wait for the hardware to finish
    while (ADCSRA & (1 << ADSC));
    
    // 5. Return the 10-bit integer
    return ADC;
}
// Initialize the QTR sensor CTRL pin (D12 / PB4) as an output
void qtr_leds_init(void) {
    // Set PB4 (D12) as an output pin
    DDRB |= (1 << DDB4); 
    
    // Start with LEDs off to save power
    PORTB &= ~(1 << PORTB4); 
}

// Turn the sensor LEDs ON
void qtr_leds_on(void) {
    PORTB |= (1 << PORTB4);
}

// Turn the sensor LEDs OFF
void qtr_leds_off(void) {
    PORTB &= ~(1 << PORTB4);
}

int main(void) {
    
    // Configure Port B, Pin 5 (Built-in LED) as an OUTPUT
    DDRB |= (1 << PORTB5);
    
    // Initialize our 500Hz timer
    encoder_init();
    motor_init ();
    adc_init();
    debug_init();
    qtr_leds_init();

    // Check the battery voltage and report it down the USB port
    long batt_reading = adc_read(7);
    long mv_read = (batt_reading * 10000L) / 1024L;

    printf("Battery = %ld mV\n", mv_read);
    
    // Read the button. Reading is near 0 when not pressed, and near 1024 when
    // pressed
    uint16_t button_val = adc_read(6);
    
    // Wait for the button to be pressed    
    while (button_val < 512)
        button_val = adc_read(6);
    // Acknowledge button press with LED and wait one second to clear hand
    led_on();
    _delay_ms(1000);
    led_off();

    // call the initialisation code in controller.c
    racer_init();
    
    // setup the 5ms controller loop
    init_timer1_200hz();
    
    // Enable global interrupts which starts the control loop
    sei();
    
    // The main loop doesn't do anything. You can put printf() calls in here to
    // see what the racer is doing if you pass the variables from controller.c.
    while (1) {
    }
    
    return 0;
}