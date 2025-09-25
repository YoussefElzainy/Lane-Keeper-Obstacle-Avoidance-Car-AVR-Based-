#define F_CPU 16000000UL
#include "motor.h"
#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "ultrasonic.h"
#include "servo.h"
// In motor.h, add these defines if not present
#define NUM_SCAN_ANGLES 5
#define SCAN_ANGLES {30, 60, 90, 120, 150}  // Right to left
#define SAFE_DISTANCE (OBSTACLE_THRESHOLD + 20)
#define TURN_INCREMENT_MS 500  // Small turn steps for adjustment
#define MAX_TURN_ATTEMPTS 15   // Prevent infinite loops
// Initialize motor control with proper PWM
void motor_init(void) {
	// Set motor control pins as outputs
	DDRB |= (1 << RIGHT_MOTOR_EN);   // PB3 as output (OC0)
	DDRA |= (1 << RIGHT_MOTOR_IN1) | (1 << RIGHT_MOTOR_IN2) |
	(1 << LEFT_MOTOR_IN3) | (1 << LEFT_MOTOR_IN4);
	DDRD |= (1 << LEFT_MOTOR_EN);    // PD7 as output (OC2)
	
	// Initialize Timer0 for PWM on right motor (PB3)
	// Fast PWM mode, non-inverting, prescaler 64
	TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS01) | (1 << CS00);
	
	// Initialize Timer2 for PWM on left motor (PD7)
	// Fast PWM mode, non-inverting, prescaler 64
	TCCR2 = (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS22);
	
	// Initialize all motors to stop
	stop_motors();
}

// Set individual motor speeds
void set_motor_speed(uint8_t left_speed, uint8_t right_speed) {
	OCR0 = right_speed;  // Right motor
	OCR2 = left_speed;   // Left motor
}

// Movement functions
void move_forward(uint8_t speed) {
	// Right motor forward
	PORTA |= (1 << RIGHT_MOTOR_IN1);
	PORTA &= ~(1 << RIGHT_MOTOR_IN2);
	
	// Left motor forward
	PORTA |= (1 << LEFT_MOTOR_IN3);
	PORTA &= ~(1 << LEFT_MOTOR_IN4);
	
	set_motor_speed(speed, speed);
	uart_puts("speed:");
	uart_puts(speed);
	uart_puts("\n");
}

void move_backward(uint8_t speed) {
	// Right motor backward
	PORTA &= ~(1 << RIGHT_MOTOR_IN1);
	PORTA |= (1 << RIGHT_MOTOR_IN2);
	
	// Left motor backward
	PORTA &= ~(1 << LEFT_MOTOR_IN3);
	PORTA |= (1 << LEFT_MOTOR_IN4);
	
	set_motor_speed(speed, speed);
		uart_puts("speed:");
		uart_puts(speed);
		uart_puts("\n");
}

void turn_left(uint8_t speed) {
	// Right motor forward
	PORTA |= (1 << RIGHT_MOTOR_IN1);
	PORTA &= ~(1 << RIGHT_MOTOR_IN2);
	
	// Left motor backward (pivot turn)
	PORTA &= ~(1 << LEFT_MOTOR_IN3);
	PORTA |= (1 << LEFT_MOTOR_IN4);
	
	set_motor_speed(0, speed);
		uart_puts("speed:");
		uart_puts(speed);
		uart_puts("\n");
}

void turn_right(uint8_t speed) {
	// Right motor backward (pivot turn)
	PORTA &= ~(1 << RIGHT_MOTOR_IN1);
	PORTA |= (1 << RIGHT_MOTOR_IN2);
	
	// Left motor forward
	PORTA |= (1 << LEFT_MOTOR_IN3);
	PORTA &= ~(1 << LEFT_MOTOR_IN4);
	
	set_motor_speed(speed, 0);
		uart_puts("speed:");
		uart_puts(speed);
		uart_puts("\n");
}

void stop_motors(void) {
	// Stop both motors
	PORTA &= ~((1 << RIGHT_MOTOR_IN1) | (1 << RIGHT_MOTOR_IN2) |
	(1 << LEFT_MOTOR_IN3) | (1 << LEFT_MOTOR_IN4));
	set_motor_speed(STOP, STOP);
		uart_puts("stopped\n");

}

// Check for obstacles using front sensor
uint8_t check_obstacle(void) {
	// Use the most recent front distance reading
	uint16_t front_dist = front_array[(front_index == 0) ? US_ARR_SIZE-1 : front_index-1];
	
	if (front_dist <= CRITICAL_DISTANCE) return 2;
	if (front_dist <= OBSTACLE_THRESHOLD) return 1;
	return 0;
}

// Improved obstacle avoidance
void avoid_obstacle(void) {
	stop_motors();
	_delay_ms(300);
	
	// Backup until front is safer
	move_backward(SLOW_SPEED);
	while (1) {
		ultrasonic_trigger_front();
		_delay_ms(50);
		uint16_t front_dist = front_array[(front_index == 0) ? US_ARR_SIZE-1 : front_index-1];
		if (front_dist > OBSTACLE_THRESHOLD) break;
		_delay_ms(200);  // Backup in small steps
	}
	stop_motors();
	_delay_ms(200);
	
	// Scan multiple angles for free space
	uint8_t angles[NUM_SCAN_ANGLES] = SCAN_ANGLES;
	uint16_t distances[NUM_SCAN_ANGLES];
	uint16_t max_dist = 0;
	uint8_t best_angle = 90;  // Default forward
	
	for (uint8_t i = 0; i < NUM_SCAN_ANGLES; i++) {
		servo_set_angle(angles[i]);
		_delay_ms(100);  // Reduced settle time; adjust as needed
		ultrasonic_trigger_front();
		_delay_ms(50);
		distances[i] = front_array[(front_index == 0) ? US_ARR_SIZE-1 : front_index-1];
		
		if (distances[i] > max_dist) {
			max_dist = distances[i];
			best_angle = angles[i];
		}
	}
	
	servo_set_angle(90);  // Reset to forward
	_delay_ms(300);
	
	// If fully blocked, turn 180°
	if (max_dist <= OBSTACLE_THRESHOLD) {
		turn_right(SLOW_SPEED);
		_delay_ms(5000);  // Calibrated 180°
		stop_motors();
		return;
	}
	
	// Turn toward best angle (left if >90, right if <90)
	uint8_t turn_dir = (best_angle > 90) ? 0 : 1;  // 0=left, 1=right
	uint16_t target_turn_ms = (abs(best_angle - 90) * 2000) / 90;  // Scale from calibrated 90° time
	
	if (turn_dir) turn_right(TURN_SPEED);
	else turn_left(TURN_SPEED);
	
	// Turn in increments with checks
	uint8_t attempts = 0;
	while (attempts < MAX_TURN_ATTEMPTS) {
		_delay_ms(TURN_INCREMENT_MS);
		stop_motors();
		_delay_ms(100);
		
		ultrasonic_trigger_front();
		_delay_ms(50);
		uint16_t current_dist = front_array[(front_index == 0) ? US_ARR_SIZE-1 : front_index-1];
		
		if (current_dist > SAFE_DISTANCE) break;  // Path clear, stop turning
		attempts++;
	}
	stop_motors();
	_delay_ms(200);
	
	// Move forward until obstacle cleared, with checks
	move_forward(NORMAL_SPEED);
	while (1) {
		ultrasonic_trigger_front();
		_delay_ms(50);
		uint16_t front_dist = front_array[(front_index == 0) ? US_ARR_SIZE-1 : front_index-1];
		if (front_dist <= OBSTACLE_THRESHOLD) {
			stop_motors();
			avoid_obstacle();  // Recursive if new obstacle (careful with stack)
			return;
		}
		if (front_dist > SAFE_DISTANCE * 2) break;  // Well clear
		_delay_ms(200);  // Move in steps
	}
	stop_motors();
	_delay_ms(200);
}