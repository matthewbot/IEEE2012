#ifndef UART_H
#define UART_H

void uart_init();

enum UARTNum {
	UART_USB,
	UART_XBEE
};

bool uart_put(UARTNum num, char ch);
int uart_puts(UARTNum num, const char *buf);
int uart_get(UARTNum num);

#endif
