#ifndef CONTROLPANEL_H_
#define CONTROLPANEL_H_

#include <stdint.h>

void controlpanel_init();
void controlpanel();
void controlpanel_motor();
void controlpanel_temp();
void controlpanel_lineSensorUpdate(const uint16_t *readings);
void controlpanel_linesensor();

#endif /* CONTROLPANEL_H_ */
