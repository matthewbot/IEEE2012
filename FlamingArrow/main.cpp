#include "init.h"
#include "controlpanel.h"
#include "linefollow.h"

int main() {
	init();
	
	while (true)
		linefollow_setEnabled(true);
}
