/*
 * uart.c
 *
 * Created: 9/11/2025 4:20:59 PM
 *  Author: Lenovo
 */ 
#include "uart.h"
#include <avr/io.h>

// ==========================
// UART Init (9600 baud, 16 MHz)
// ==========================
void uart_init(void) {
uint16_t ubrr = 103; // 9600 baud at 16 MHz
UBRRH = (ubrr >> 8);
UBRRL = ubrr;
UCSRB = (1 << TXEN);
UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);

}

// Send single char
void uart_putc(char c) {
	while (!(UCSRA & (1 << UDRE))); // Wait for buffer empty
	UDR = c;
}

// Send string
void uart_puts(const char *s) {
	while (*s) {
		uart_putc(*s++);
	}
}

// Send integer as ASCII
void uart_putint(uint16_t val) {
	char buf[10];
	itoa(val, buf, 10);
	uart_puts(buf);
}

void uart_send_array(uint16_t *arr, uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		uart_putint(arr[i]);   // send number
		if (i < len - 1) {
			uart_putc(',');    // comma separator
		}
	}
	uart_putc('\n');           // end line
}

void UART_sendChar(char data) {
	while (!(UCSRA & (1 << UDRE)));  // Wait for empty buffer
	UDR = data;                      // Send data
}


