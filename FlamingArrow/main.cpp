#include "init.h"
#include "debug.h"
#include "linesensor.h"
#include <util/delay.h>

int main() {
	init();

	while (true) {
		_delay_ms(10);
		
		for (int i=0; i<linesensor_count; i++)
			debug_printf("%5u,", linesensor_get(i));
		debug_putch('\n');
	}
}