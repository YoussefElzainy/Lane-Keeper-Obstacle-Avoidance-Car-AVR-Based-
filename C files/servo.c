/*
 * servo.c
 *
 * Created: 9/11/2025 4:12:29 PM
 * Author: Lenovo
 */ 
#define F_CPU 16000000UL
#include "servo.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define SERVO_MIN 1000   // 0.5 ms = 1000 ticks
#define SERVO_MAX 5000   // 2.5 ms = 5000 ticks

uint8_t servo_angle = 90; // Fixed at center position
uint8_t direction = 0; // Not used anymore

// Timer1: Servo PWM on OC1A (PD5)
void servo_init(void)
{
    DDRD |= (1 << PD5);  // PD5 = OC1A output

    // Fast PWM mode 14: ICR1 as TOP, non-inverting on OC1A
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // prescaler = 8

    ICR1 = 39999;      // TOP = 39999 ? 20 ms frame
    OCR1A = servo_angle_to_ticks(servo_angle); // Start at center
}

// Map angle to ticks
uint16_t servo_angle_to_ticks(uint8_t angle)
{
    return SERVO_MIN + ((uint32_t)angle * (SERVO_MAX - SERVO_MIN)) / 180;
}

void servo_set_angle(uint8_t angle)
{
    // Limit angle to valid range
    if (angle > 180) angle = 180;
    if (angle < 0) angle = 0;
    
    servo_angle = angle;
    OCR1A = servo_angle_to_ticks(angle);
}

// Empty function since we're not sweeping anymore
void update_servo(void){
    // Do nothing - servo stays at fixed position
}