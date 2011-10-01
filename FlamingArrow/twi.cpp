#include <avr/io.h>

#include "twi.h"

static TWI_t &t = TWIC;
#define twi t.MASTER
#define MASTER_CLOCK 32000000
#define SLAVE_CLOCK 100000

void twi_init() {
	twi.BAUD = (uint8_t) (MASTER_CLOCK/(2*SLAVE_CLOCK)-5);
	twi.CTRLA = TWI_MASTER_INTLVL_HI_gc | TWI_MASTER_ENABLE_bm | TWI_MASTER_RIEN_bm;
}

void twi_start_addr(register8_t addr, Direction direction) { // Start bit, address, and R/W bit
	twi.ADDR = (addr << 1) | direction;
}

bool twi_ack() {
	if (twi.STATUS == TWI_MASTER_RXACK_bm) // Acknowledge bit not received
		return false;
	else return true;
}

void twi_write(uint8_t data) {
	twi.DATA = data;
}

void twi_repeated_start() {
	twi.CTRLC = TWI_MASTER_CMD_REPSTART_gc; // Write to CTRLC CMD bits to send a repeated start
}

uint8_t twi_get() {
	return twi.DATA;
}

void twi_send_ack() {
	twi.CTRLC = TWI_MASTER_CMD1_bm; // Send ack and wait for next data byte
}

void twi_send_ack_stop() {
	twi.CTRLC = TWI_MASTER_CMD1_bm | TWI_MASTER_CMD0_bm; // Send ack and stop
}

void twi_clear_intflags() {
	twi.STATUS = TWI_MASTER_RIF_bm | TWI_MASTER_WIF_bm;
}

bool twi_intflag() {
	if (twi.STATUS & TWI_MASTER_WIF_bm)
		return true;
	if (twi.STATUS & TWI_MASTER_RIF_bm)
		return true;
	else
		return false;
}
