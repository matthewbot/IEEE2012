#ifndef SENSORS_H_
#define SENSORS_H_

void sensors_init();

bool sensors_readBump();

float sensors_readCapADC();
float sensors_readVoltageADC();
float sensors_readSignalADC();

#endif

