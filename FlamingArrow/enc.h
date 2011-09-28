#ifndef ENC_H_
#define ENC_H_

#include <stdint.h>

void enc_init();

static const int enc_count = 2;

int16_t enc_get(uint8_t num);
void enc_reset(uint8_t num);

#endif /* ENC_H_ */
