#include "init.h"
#include "controlpanel.h"

int main() {
	init();
	
	while (true)
		controlpanel();
}
