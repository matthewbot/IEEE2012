#include <avr/io.h>
#include "temp.h"
#include "twi.h"
#include <avr/interrupt.h>

#define TEMP_AMB_REG 0x06
#define TEMP_OBJ_REG 0x07
#define TWI_INT_VEC TWIC_TWIM_vect

const uint8_t temp_addr = 0x5A;

static uint8_t ambient_temp = 0;
volatile static int count = 0;
volatile static bool got = false;

uint16_t temp_getraw(Temp_type temp_type) {

	uint16_t current_temp = 0;

	twi_start_addr(temp_addr, WRITE);
	while(!twi_ack()) {}; // Wait for acknowledge

	switch(temp_type) {
		case AMB:
			twi_write(TEMP_AMB_REG);
			break;
		case OBJ:
			twi_write(TEMP_OBJ_REG);
			break;
	}
	while(!twi_ack()) {}; // Wait for acknowledge

	twi_repeated_start();

	twi_start_addr(temp_addr, READ);
	while(!twi_ack()) {}; // Wait for acknowledge

	while(!twi_intflag()) {}; // Wait to get a temperature reading
	current_temp = twi_get() << 8;
	twi_clear_intflags();

	twi_send_ack();
	current_temp |= twi_get();

	twi_send_ack_stop();

	return current_temp;
}

float temp_get(Temp_type temp_type) {
	uint16_t sum_amb = 0, sum_obj = 0;

	for (int i=0; i<10; i++) {
		sum_amb += temp_getraw(AMB)*0.02 - 273.15;
		sum_obj += temp_getraw(OBJ)*0.02 - 273.15;
	}

	switch(temp_type) {
		case AMB:
			return sum_amb/10.0;
			break;
		case OBJ:
			return sum_obj/10.0;
			break;
	}
}

float temp_getdifference() {
	return (temp_get(OBJ) - temp_get(AMB));
}
