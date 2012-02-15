#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>

void sensors_init();

bool sensors_readBump();

uint16_t sensors_readCapADC();
uint16_t sensors_readVoltageADC();
uint16_t sensors_readSignalADC();//hey matt! Long bathroom break, eh??

enum SensorConfig {
	SENSOR_MEASURE,
	SENSOR_CHARGE,
	SENSOR_DISCHARGE
};

void sensors_config(SensorConfig config);

#endif

