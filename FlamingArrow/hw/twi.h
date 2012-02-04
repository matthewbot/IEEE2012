#ifndef TWI_H_
#define TWI_H_

#include <stdint.h>

#include <avr/io.h>

enum Direction {
	DIR_WRITE,
	DIR_READ
};

void twi_init();
bool twi_start(uint8_t addr, Direction dir);
void twi_stop();

uint8_t twi_read();
void twi_ack();
void twi_write(uint8_t data);

#endif /* TWI_H_ */
