
#ifndef TWI_H_
#define TWI_H_

enum Direction {
	WRITE,
	READ	
};

void twi_init();
void twi_start_addr(register8_t, Direction);
bool twi_ack();
void twi_repeated_start();
uint8_t twi_get();
void twi_write(uint8_t);
void twi_send_ack();
void twi_send_ack_stop();
void twi_clear_intflags();
void twi_intflag();

#endif /* TWI_H_ */