#include <util/delay.h>

#include "init.h"
#include "linefollow.h"

int main() {
	init();
	
	linefollow_setEnabled(true);
	
	while (true);
}
