#include "control/deploy.h"
#include "hw/motor.h"
#include "hw/adc.h"

static const int16_t deploy_pwm = 700;

static volatile float beambreak_filtered = 0;
static volatile bool enabled;
static volatile bool seen_beambreak;

void deploy_out(bool full) {
	motor_setpwm(MOTOR_DEPLOY, full ? motor_maxpwm : deploy_pwm);
}

void deploy_in(bool full) {
	motor_setpwm(MOTOR_DEPLOY, full ? -motor_maxpwm : -deploy_pwm);
}

void deploy_off() {
	motor_setpwm(MOTOR_DEPLOY, 0);
}

bool deploy_getBeamBreak() {
	return beambreak_filtered > 3000;
}

void deploy_start() {
	seen_beambreak = false;
	deploy_out();
	enabled = true;
}

void deploy_stop() {
	enabled = false;
	deploy_off();
}

bool deploy_isDone() {
	return !enabled;
}

void deploy_tick() {
	beambreak_filtered = .9f*beambreak_filtered + .1f*adc_sample(ADC_BEAM_BREAK);
	
	if (!enabled)
		return;
		
	bool beambreak = deploy_getBeamBreak();
	if (!seen_beambreak) {
		if (beambreak)
			seen_beambreak = true;
	} else {
		if (!beambreak)
			deploy_stop();
	}
}
	
