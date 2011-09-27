#include "init.h"
#include "debug.h"
#include "linesensor.h"
#include "temp.h"
#include <util/delay.h>

int main() {
	init();
	controlpanel();

	while (true) {
		_delay_ms(10);ain() {
	init();
	pid_obj_t p;
	pid_new(&p, .8, .1, .4, .01);
	float t = 0;

	while (true) {
		debug_printf("\r\n");
		int16_t e0 = enc_get(0);
		float rps = e0/563.03/.05;
		debug_printf("enc: %i rps: %f\r\n", e0, (double)rps);
		debug_printf("int: %f\r\n", (double)p.i_sum);
		
		float out = pid_update(&p, 1.3*sin(t), rps, .05);
		debug_printf("out: %f\r