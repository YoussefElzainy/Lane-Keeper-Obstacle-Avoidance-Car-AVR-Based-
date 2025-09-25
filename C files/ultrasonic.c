#include "ultrasonic.h"
#include <util/delay.h>

// Define CPU frequency for delay.h
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

volatile uint8_t front_done = 0;
volatile uint32_t front_time = 0;

uint16_t front_array[US_ARR_SIZE];
uint8_t front_index = 0;

// Internal flags
volatile uint8_t front_rising = 0;
volatile uint32_t front_start = 0;
volatile uint16_t front_overflow_count = 0;

// Timer1 overflow counter for extended time measurement
ISR(TIMER1_OVF_vect)
{
	if (front_rising) front_overflow_count++;
}

void ultrasonic_init(void)
{
	// Trigger as output
	DDRA |= (1 << US_FRONT_TRIG);
	
	// Echo pin as input
	DDRD &= ~(1 << US_FRONT_ECHO);
	
	// Enable pull-up resistor on echo pin
	PORTD |= (1 << US_FRONT_ECHO);

	// INT0 rising edge initially
	MCUCR |= (1 << ISC01) | (1 << ISC00); // rising edge for INT0

	GICR |= (1 << INT0);    // enable INT0
	
	// Enable Timer1 overflow interrupt for extended time measurement
	TIMSK |= (1 << TOIE1);
}

// Trigger front ultrasonic (10 us pulse)
void ultrasonic_trigger_front(void)
{
	// Ensure echo line is low before triggering
	while (PIND & (1 << US_FRONT_ECHO)) { /* wait */ }
	
	PORTA |= (1 << US_FRONT_TRIG);
	_delay_us(15);            // 15 µs pulse
	PORTA &= ~(1 << US_FRONT_TRIG);
	front_done = 0;
	front_rising = 0;
	front_overflow_count = 0;
}

// Calculate time difference considering overflows
uint32_t calculate_time_difference(uint32_t start, uint32_t end, uint16_t overflow_count)
{
	if (end >= start) {
		return (end - start) + (overflow_count * 65536UL);
		} else {
		return ((65536UL - start) + end) + (overflow_count * 65536UL);
	}
}

// INT0 ISR (Front Echo)
ISR(INT0_vect)
{
	if (PIND & (1 << US_FRONT_ECHO)) { // Rising edge
		front_start = TCNT1;
		front_rising = 1;
		front_overflow_count = 0;
		MCUCR &= ~(1 << ISC00); // Set for falling edge next
		} else { // Falling edge
		if (front_rising) {
			uint32_t now = TCNT1;
			front_time = calculate_time_difference(front_start, now, front_overflow_count);
			
			// Convert to cm: time in ticks * 0.5µs / 58
			uint32_t distance_cm = front_time / 116;
			
			if (distance_cm > 400) distance_cm = 400;
			
			front_array[front_index] = (uint16_t)distance_cm;
			front_index = (front_index + 1) % US_ARR_SIZE;
			front_done = 1;
		}
		front_rising = 0;
		MCUCR |= (1 << ISC00); // Set back to rising edge
	}
}