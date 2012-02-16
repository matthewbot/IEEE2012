#ifndef CONTROLPANEL_H_
#define CONTROLPANEL_H_

#include "control/pid.h"
#include <stdint.h>

void controlpanel_init();

void controlpanel();
void controlpanel_sensor();
void controlpanel_drive();
void controlpanel_tests();

bool controlpanel_promptGains(const char *name, const PIDGains &curgains, PIDGains &gains);
int controlpanel_prompt(const char *prompt, const char *fmt, ...);
char controlpanel_promptChar(const char *prompt);

#endif /* CONTROLPANEL_H_ */