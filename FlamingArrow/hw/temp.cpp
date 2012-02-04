#include "hw/twi.h"
#include "hw/temp.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

#define TEMP_AMB_REG 0x06
#define TEMP_OBJ_REG 0x07
#define TWI_INT_VEC TWIC_TWIM_vect

static const uint8_t temp_addr = 0x5A;

static void debug(const char *s) {
	printf("%s. STATUS %x\n", s, TWIE.MASTER.STATUS);
}

bool temp_getROM16(uint8_t reg, uint16_t &val) {
	return temp_get16(reg | 0x20, val);
}

bool temp_get16(uint8_t reg, uint16_t &val) {
	printf("Baud %u\n", (unsigned int)TWIE.MASTER.BAUD);
	
	//debug("Sending start");
	if (!twi_start(temp_addr, DIR_WRITE)) {
		debug("Start fail");
		return false;
	}
	
	//debug("Writing register");
	twi_write(reg);
	
	TWIE.MASTER.CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	
	//debug("Repeated start");
	if (!twi_start(temp_addr, DIR_READ)) {
		debug("Repeat start fail");
		return false;
	}
	
	//debug("Reading byte 0");
	val = twi_read() << 8;
	//debug("Sending ack");
	twi_ack();
	//debug("Reading byte 1");
	val |= twi_read();
	//debug("Sending stop");
	twi_stop();
	
	printf("Done!");
	return true;
}

