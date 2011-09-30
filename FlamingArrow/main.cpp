#include "init.h"
#include "debug.h"
#include "linesensor.h"
#include <util/delay.h>
#include "motor.h"
#include "enc.h"
#include "linefollow.h"

int main() {
	init();
	
	linefollow_setEnabled(true);

	while (true);
}
