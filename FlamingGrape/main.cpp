#include "init.h"
#include "debug/controlpanel.h"
#include "competition/nav.h"

int main() {
	init();

	while (true) {
		bool go = controlpanel();
		if (go)
			nav_go();
	}
}
