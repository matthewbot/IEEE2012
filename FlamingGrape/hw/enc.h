#ifndef ENC_H_
#define ENC_H_

#include <stdint.h>

static const float enc_per_rotation = 1920;

void enc_init();
uint16_t enc_get(uint8_t num);
void enc_reset(uint8_t num);

int16_t enc_diff(uint16_t a, uint16_t b);

#endif /* ENC_H_ */
