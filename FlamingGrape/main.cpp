#include "init.h"
#include "debug/controlpanel.h"

int main() {
	init();

	while (true)
		controlpanel();
}
