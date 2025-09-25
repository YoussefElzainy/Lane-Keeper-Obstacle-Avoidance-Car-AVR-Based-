
/*
 * main.c
 *
 * Created: 9/8/2025 9:01:47 AM
 * Author : Lenovo
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h> // Added for snprintf

#include "servo.h"
#include "uart.h"
#include "ultrasonic.h"
#include "motor.h"

// Timing constants
#define OBSTACLE_CHECK_INTERVAL 150  // Check every 150ms
#define STOP_MS 100                  // Stop duration to kill momentum
#define TURN_PULSE_MS 200            // Pulse duration for turn
#define PULSE_STOP_MS 10             // Stop between pulses
#define BUZZER_MS 100                // Buzzer duration
#define REVERSE_PULSE_MS 300         // Pulse duration for reverse turn
#define REVERSE_PULSES 1             // Number of reverse pulses to center
#define MAX_TURN_ATTEMPTS 100        // Prevent infinite loops

// Global Variables
volatile uint32_t ms_counter = 0;
volatile uint8_t is_avoiding = 0;
volatile uint8_t line_detected = 0;  // 0=none, 1=left (PC0), 2=right (PC1)
extern uint16_t front_array[];
extern uint8_t front_index;
#define US_ARR_SIZE 12

// State machine for line correction
typedef enum {
    IDLE,
    STOPPING,
    BUZZING,
    TURNING,
    REVERSING
} CorrectionState;

volatile CorrectionState corr_state = IDLE;
volatile uint32_t state_start_time = 0;
volatile uint8_t turn_attempts = 0;
volatile uint8_t reverse_count = 0;
volatile uint8_t turn_dir = 0;  // 0=left, 1=right

// Timer2 overflow for millis
ISR(TIMER2_OVF_vect) {
    ms_counter++;
}

// INT2 ISR: Set flags based on IR inputs
ISR(INT2_vect) {
    if (!is_avoiding) {
        uint8_t pc0_state = (PINC & (1 << PC0)) ? 1 : 0;  // low = tape
        uint8_t pc1_state = (PINC & (1 << PC1)) ? 1 : 0;

        if (!pc0_state && line_detected == 0) {
            line_detected = 1;  // Left tape detected
        } else if (!pc1_state && line_detected == 0) {
            line_detected = 2;  // Right tape detected
        }
    }
}

uint32_t millis(void) {
    uint32_t snapshot;
    cli();
    snapshot = ms_counter;
    sei();
    return snapshot;
}

void timer2_init(void) {
    TIMSK |= (1 << TOIE2);
    sei();
}

void int2_init(void) {
    // Set PC0 and PC1 as inputs
    DDRC &= ~((1 << PC0) | (1 << PC1));
    // Enable pull-up resistors on PC0 and PC1
    PORTC |= (1 << PC0) | (1 << PC1);
    
    // Set PC3 as output for buzzer
    DDRC |= (1 << PC3);
    
    // Configure INT2 for rising edge
    MCUCSR |= (1 << ISC2);
    GICR |= (1 << INT2);  // Enable INT2
}

void handle_line_correction(void) {
    // Use TURN_SPEED if defined, else NORMAL_SPEED * 1.5 (cap at 255)
    uint8_t turn_speed = NORMAL_SPEED;
    #ifdef TURN_SPEED
    turn_speed = TURN_SPEED;
    #else
    if (turn_speed < 200) turn_speed = 200;
    #endif

    uint32_t now = millis();
    uint8_t pc0_state = (PINC & (1 << PC0)) ? 1 : 0;  // low = tape
    uint8_t pc1_state = (PINC & (1 << PC1)) ? 1 : 0;

    switch (corr_state) {
        case IDLE:
            if (line_detected == 1 || line_detected == 2) {
                uart_puts(line_detected == 1 ? "Left tape detected\n" : "Right tape detected\n");
                stop_motors(); 
                turn_dir = (line_detected == 1) ? 1 : 0;  // 0=left, 1=right
                state_start_time = now;
                corr_state = STOPPING;
            }
            break;

        case STOPPING:
            if (now - state_start_time >= STOP_MS) {
                uart_puts("Stop complete\n");
                PORTC |= (1 << PC3);  // Beep
                state_start_time = now;
                corr_state = BUZZING;
            }
            break;

        case BUZZING:
            if (now - state_start_time >= BUZZER_MS) {
                PORTC &= ~(1 << PC3);  // Stop beep
                uart_puts("Buzz complete\n");
                uart_puts("Starting turn\n");
                // Start turning
                if (turn_dir) turn_right(turn_speed);
                else turn_left(turn_speed);
                state_start_time = now;
                turn_attempts = 1;
                corr_state = TURNING;
            }
            break;

        case TURNING:
            // Check if IR is still triggered (low)
            if ((line_detected == 1 && !pc0_state) || (line_detected == 2 && !pc1_state)) {
                // Continue turning
                if (turn_dir) turn_right(turn_speed);
                else turn_left(turn_speed);
                if (turn_attempts >= MAX_TURN_ATTEMPTS) {
                    uart_puts("Max turn attempts reached\n");
                    corr_state = REVERSING;  // Force reverse
                    state_start_time = now;
                    reverse_count = 0;
                }
                turn_attempts++;
            } else {
                // IR released (high), stop turning and reverse
                stop_motors();
                uart_puts("Line lost, reversing\n");
                if (!turn_dir) turn_right(turn_speed);  // Opposite direction
                else turn_left(turn_speed);
                state_start_time = now;
                reverse_count = 0;
                corr_state = REVERSING;
            }
            break;

        case REVERSING:
            if (now - state_start_time >= REVERSE_PULSE_MS) {
                stop_motors();
                reverse_count++;
                if (reverse_count >= REVERSE_PULSES) {
                    uart_puts("Correction complete\n");
                    move_forward(NORMAL_SPEED);
                    line_detected = 0;
                    corr_state = IDLE;
                } else {
                    if (!turn_dir) turn_right(turn_speed);
                    else turn_left(turn_speed);
                    state_start_time = now;
                    corr_state = REVERSING;
                }
            }
            break;
    }
}

void send_to_esp32(const char *message) {
    uart_puts(message);
    uart_putc('\n');
}

int main(void) {
    servo_init();
    ultrasonic_init();
    motor_init();
    timer2_init();
    int2_init();
    uart_init();

    uint32_t last_obstacle_check = 0;

    // Set servo to look forward (90 degrees)
    servo_set_angle(90);
    _delay_ms(500);

    // Start moving forward
    move_forward(NORMAL_SPEED);

    uart_puts("System started - Obstacle avoidance and line correction mode\r\n");

    while (1) {
        uint32_t now = millis();

        // Handle line corrections
        if (!is_avoiding) {
            handle_line_correction();
        }

        // Check for obstacles
        if (!is_avoiding && (now - last_obstacle_check) >= OBSTACLE_CHECK_INTERVAL) {
            last_obstacle_check = now;
            
            // Trigger front ultrasonic and wait for measurement
            ultrasonic_trigger_front();
            _delay_ms(50);
            
            uint8_t obstacle_status = check_obstacle();
            
            if (obstacle_status > 0) {
                is_avoiding = 1;
                line_detected = 0;  // Reset during avoidance
                corr_state = IDLE;  // Reset correction state
                uart_puts("OBSTACLE! Avoiding\n");
                avoid_obstacle();
                uart_puts("Avoidance complete\n");
                is_avoiding = 0;
                
                // Return servo to forward position
                servo_set_angle(90);
                _delay_ms(300);
                
                // Resume forward movement
                move_forward(NORMAL_SPEED);
            }
        }
        
        static uint32_t last_distance_send = 0;
        if (millis() - last_distance_send > 2000) {
            last_distance_send = millis();
            // Get the most recent distance reading
            uint16_t current_distance = front_array[(front_index == 0) ? US_ARR_SIZE-1 : front_index-1];
            char dist_msg[30];
            snprintf(dist_msg, sizeof(dist_msg), "Distance: %d cm", current_distance);
            send_to_esp32(dist_msg);
        }
        // Minimal delay for loop
        _delay_ms(2);
    }
}
