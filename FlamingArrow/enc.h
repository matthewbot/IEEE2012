#ifndef ENC_H_
#define ENC_H_

#include <stdint.h>

static const int enc_count = 2;

void enc_init();
uint16_t enc_get(uint8_t num);
void enc_reset(uint8_t num);

#endif /* ENC_H_ */
