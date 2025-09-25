/*
 * uart.h
 *
 * Created: 9/11/2025 4:21:13 PM
 *  Author: Lenovo
 */ 
#include <stdint.h>   // for uint8_t, uint16_t

#ifndef UART_H_
#define UART_H_

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_putint(uint16_t val);
void uart_send_array(uint16_t *arr, uint8_t len);


#endif /* UART_H_ */