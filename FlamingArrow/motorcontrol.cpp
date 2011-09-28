#include "motorcontrol.h"
#include "motor.h"
#include "pid.h"
#include "enc.h"
#include <avr/io.h>
#include <avr/interrupt.h>

static TC1_t &pidtim = TCF1;
#define TIMOVFVEC TCF1_OVF_vect

static const float ticks_per_rotation = 563.03;
static const float update_hz = 20;

struct MotorInfo {
	pid_obj_t pid;
	float rps_desired;
	float rps_measured;
	bool enabled;
};

static MotorInfo motinfo[2];

void motorcontrol_init() {
	for (int i=0; i<2; i++)
		pid_new(&motinfo[i].pid, .8, .1, .4, .01);

	pidtim.CTRLA = TC_CLKSEL_DIV64_gc; // timer runs at 1Mhz
	pidtim.INTCTRLA = TC_OVFINTLVL_MED_gc; // enable medium priority interrupt
	pidtim.PER = 500000 / update_hz; // period computed from update_hz
}

float motorcontrol_getvel(int mot) {
	return motinfo[mot].rps_measured;
}

void motorcontrol_setvel(int mot, float rps) {
	motinfo[mot].rps_desired = rps;
	motinfo[mot].enabled = true;
}

void motorcontrol_disable(int mot) {
	motinfo[mot].enabled = false;
	motinfo[mot].rps_measured = 0;
	motor_setpwm(mot, 0);
}

ISR(TIMOVFVEC) {
	for (int motnum=0; motnum<2; motnum++) { // for each motor
		MotorInfo &mot = motinfo[motnum]; // get its motor information

		if (!mot.enabled) // if its not enabled
			continue; // skip it

		int16_t enc = enc_get(motnum); // read the amount the motor traveled since the last update
		enc_reset(motnum); // reset to begin measuring for the next cycle

		mot.rps_measured = enc/ticks_per_rotation*update_hz; // compute the rotations per second

		float output = pid_update(&mot.pid, mot.rps_desired, mot.rps_measured, 1/update_hz); // update the PID loop
		if (output > 1) // enforce saturation on the output
			output = 1;
		else if (output < -1)
			output = -1;

		motor_setpwm(motnum, (int16_t)(output*motor_maxpwm)); // convert output to pwm, set it to the motor
	}
}

