#include "competition/sensordecision.h"
#include "competition/sensorcomms.h"
#include "debug/debug.h"
#include "hw/tick.h"
#include <avr/pgmspace.h>
#include <stdio.h>

static BoardNum curboard;
static bool random;

void sensordecision_setRandomMode(bool newrandom) {
	random = newrandom;
}

bool sensordecision_verifyOk() {
	if (random)
		return true;
	
	bool ok = true;
	
	for (int i=0; i<BOARDNUM_MAX; i++) {
		if (sensorcomms_getBoardStatus((BoardNum)i) != BOARDSTATUS_READY) {
			debug_setLED(ERROR_LED, true);
			printf_P(PSTR("Board %d not ready\n"), i);
			ok = false;
			continue;
		}
		
		sensorcomms_updateBoard((BoardNum)i);
		if (!sensorcomms_waitBoard((BoardNum)i, 1000)) {
			debug_setLED(ERROR_LED, true);
			printf_P(PSTR("Board %d did not respond to an update\n"), i);
			ok = false;
			continue;
		}
	}
	
	return ok;
}

void sensordecision_prepare(uint8_t decision) {
	printf_P(PSTR("Prepare %d\n"), decision);
	
	if (random)
		return;
		
	curboard = (BoardNum)decision;
	sensorcomms_updateBoard(curboard);
}

bool sensordecision_available() {
	if (random)
		return true;
	return sensorcomms_getBoardStatus(curboard) == BOARDSTATUS_READY;
}

bool sensordecision_wait() {
	if (random)
		return true;
	
	return sensorcomms_waitBoard(curboard, 1000);
}

bool sensordecision_isRight() {
	if (random) {
		return (tick_getCount() & 0x01) != 0;
	}
	
	if (!sensordecision_available()) {
		printf_P(PSTR("Decision not available, going right\n"));
		return true;
	}
		
	const uint8_t *data = sensorcomms_getBoardReading(curboard);
	
	switch (curboard) {
		case BOARDNUM_VOLTAGE: {
			uint16_t voltage = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
			printf_P(PSTR("Voltage %x\n"), voltage);
			return voltage > 0x0283;		// Calibrated Value
		}
			
		case BOARDNUM_CAPACITANCE: {
			uint16_t volt3 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
			printf_P(PSTR("Capacitance %x\n"), volt3);
			return volt3 < 0x0100;
		}
		
		case BOARDNUM_TEMPERATURE: {
			uint16_t dir = ((uint16_t)data[2] << 8) | (uint16_t)data[3];
			return dir == 1;
		}
		
		case BOARDNUM_SIGNAL: {
			uint8_t ctr=0;
			for (int i=0;i<10;i++) {
				uint16_t val = ((uint16_t)data[2*i] << 8) | (uint16_t)data[2*i+1];
				if (val > 0x20)
					ctr++;
			}
			
			printf("Signal ctr: %d\n", ctr);
			
			return ctr < 3;
		}
		
		default:
			return false;
	}
}
