#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// Config - Only using front sensor
#define US_FRONT_TRIG   PA0  // PA0
#define US_FRONT_ECHO   PD2  // INT0

#define US_ARR_SIZE 12

// Globals
extern volatile uint8_t front_done;
extern volatile uint32_t front_time;
extern uint16_t front_array[US_ARR_SIZE];
extern uint8_t front_index;

// Functions
void ultrasonic_init(void);
void ultrasonic_trigger_front(void);

#endif /* ULTRASONIC_H_ */