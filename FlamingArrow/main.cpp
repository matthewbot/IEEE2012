#include "init.h"
#include "debug.h"
#include "linesensor.h"
#include <util/delay.h>

int main() {
	init();

	while (true) {
		_delay_ms(10);
		
	        debug_printf("%u %u %u %u %u %u %u %u\r\n",
        	        linesensor_get(0),
                	linesensor_get(1),
	                linesensor_get(2),
        	        linesensor_get(3),
                	linesensor_get(4),
	                linesensor_get(5),
        	        linesensor_get(6),
                	linesensor_get(7)
	        );
	}
}