#ifndef TEMP_H_
#define TEMP_H_

enum Temp_type {
	AMB,
	OBJ	
};	

uint16_t temp_getraw(Temp_type);
float temp_get(Temp_type);
float temp_getdifference();

#endif /* TEMP_H_ */