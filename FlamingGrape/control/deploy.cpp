#include "control/deploy.h"
#include "hw/motor.h"
#include "hw/adc.h"

static const int16_t deploy_pwm = 700;

enum State {
	STATE_ENTER_BEAM,
	STATE_LEAVE_BEAM,
	STATE_WAIT_CTR
};

static volatile State state;
static volatile float beambreak_filtered = 0;
static volatile uint8_t waitctr;
static volatile bool enabled;

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
	deploy_out();
	enabled = true;
	state = STATE_ENTER_BEAM;
}

void deploy_stop() {
	enabled = false;
	deploy_off();
}

bool deploy_isDone() {
	return !enabled;
}

void deploy_waitDone() {
	while (enabled) { }
}

void deploy_tick() {
	beambreak_filtered = .9f*beambreak_filtered + .1f*adc_sample(ADC_BEAM_BREAK);
	
	if (!enabled)
		return;
		
	bool beambreak = deploy_getBeamBreak();
	switch (state) {
		case STATE_ENTER_BEAM:
			if (beambreak)
				state = STATE_LEAVE_BEAM;
			break;
			
		case STATE_LEAVE_BEAM:
			if (!beambreak) {
				waitctr = 150;
				state = STATE_WAIT_CTR;
			}
			break;
		
		case STATE_WAIT_CTR:
			if (waitctr-- == 0)
				deploy_stop();
			break;
	}
}
	
