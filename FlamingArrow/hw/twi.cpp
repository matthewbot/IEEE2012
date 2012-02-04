#include "hw/twi.h"
#include "debug/debug.h"

#include <util/delay.h>
#include <avr/io.h>
#include <stdio.h>

static TWI_t &twi = TWIE;
static TWI_MASTER_t &twi_m = twi.MASTER;
#define MASTER_CLOCK 32000000
#define SLAVE_CLOCK 100000

void twi_init() {
	//twi_m.BAUD = (uint8_t)(MASTER_CLOCK/(2*SLAVE_CLOCK) - 5);
	twi_m.BAUD = 250;
	twi_m.CTRLA = TWI_MASTER_ENABLE_bm;
	twi_m.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
}

bool twi_start(uint8_t addr, Direction dir) { // Start bit, address, and R/W bit
	twi_m.ADDR = (addr << 1) | (dir&0x1);
	
	while ((twi_m.STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_RIF_bm)) == 0) { }

	if ((twi_m.STATUS & (TWI_MASTER_RXACK_bm | TWI_MASTER_ARBLOST_bm)) == 0) {
		return true;
	} else {
		twi_m.CTRLC = TWI_MASTER_CMD_STOP_gc;
		return false;
	}
}

uint8_t twi_read() {
	while ((twi_m.STATUS & TWI_MASTER_RIF_bm) == 0) { }
	return twi_m.DATA;
}

void twi_ack() {
	twi_m.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;	
}

void twi_write(uint8_t data) {
	while ((twi_m.STATUS & TWI_MASTER_WIF_bm) == 0) { }
	twi_m.DATA = data;
}

void twi_stop() {
	twi_m.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
}
