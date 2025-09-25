/*
 * servo.h
 *
 * Created: 9/11/2025 4:13:29 PM
 *  Author: Lenovo
 */ 
#include <stdint.h>   // for uint8_t, uint16_t
#include <avr/io.h>   // for DDRx, PORTx, etc.

#ifndef SERVO_H_
#define SERVO_H_

// External declarations for global variables
extern uint8_t servo_angle;
extern uint8_t direction;

void servo_init(void);
uint16_t servo_angle_to_ticks(uint8_t angle);
void servo_set_angle(uint8_t angle);
void update_servo(void);
void UART_sendString(const char *str);

#endif /* SERVO_H_ */