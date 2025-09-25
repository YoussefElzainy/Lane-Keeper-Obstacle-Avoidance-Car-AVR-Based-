#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

// Motor pin definitions
#define RIGHT_MOTOR_EN   PB3  // OC0 - Timer0 PWM
#define RIGHT_MOTOR_IN1  PA3
#define RIGHT_MOTOR_IN2  PA2

#define LEFT_MOTOR_EN    PD7  // OC2 - Timer2 PWM
#define LEFT_MOTOR_IN3   PA5
#define LEFT_MOTOR_IN4   PA4

// Speed definitions
#define NORMAL_SPEED     170
#define SLOW_SPEED       150
#define TURN_SPEED       255
#define STOP             0

// Distance threshold for obstacle detection (in cm)
#define OBSTACLE_THRESHOLD 30
#define CRITICAL_DISTANCE 15

// Function prototypes
void motor_init(void);
void set_motor_speed(uint8_t left_speed, uint8_t right_speed);
void move_forward(uint8_t speed);
void move_backward(uint8_t speed);
void turn_left(uint8_t speed);
void turn_right(uint8_t speed);
void stop_motors(void);
uint8_t check_obstacle(void);
void avoid_obstacle(void);

#endif /* MOTOR_H_ */