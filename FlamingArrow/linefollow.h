#ifndef LINEFOLLOW_H_
#define LINEFOLLOW_H_

#include <stdint.h>

void linefollow_init();
void linefollow_setEnabled(bool enbld=true);
void linefollow_sensorUpdate(const uint16_t *readings);

#endif /* LINEFOLLOW_H_ */
