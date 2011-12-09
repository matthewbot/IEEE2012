#define __STDC_LIMIT_MACROS

#include <stdio.h>

#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include <math.h>

#include "hw/enc.h"
#include "hw/motor.h"
#include "pid.h"
#include "util.h"
#include "control/motorcontrol.h"

static TC1_t &pidtim = TCF1;
#define TIMOVFVEC TCF1_OVF_vect

static const float ticks_per_rotation = 2500; // accurate
static const float update_hz = 50;
static const PIDCoefs pidcoefs = { 1, 1, .003*0, .01 };

volatile static bool enabled;
volatile static bool debug;

struct MotorInfo {
	PIDState pid; // state of PID loop
	float m;	// slope for open-loop lookup
	float b;	// intercept for open-loop lookup
	volatile float rps_desired;
	volatile float rps_measured;
	volatile uint16_t prev_enc; // used to find motor velocity
};

static MotorInfo motinfo[motor_count];

void motorcontrol_init() {
	pidtim.CTRLA = TC_CLKSEL_DIV64_gc; // timer runs at 1Mhz
	pidtim.INTCTRLA = TC_OVFINTLVL_MED_gc; // enable medium priority overflow interrupt
	pidtim.PER = 500000 / update_hz; // period computed, so overflow freq is update_hz
	motinfo[0].m = 78.1250;
	motinfo[0].b = 577.125;
	motinfo[1].m = 78.1250;
	motinfo[1].b = 577.125;
}

float motorcontrol_getrps(int motnum) {
	return motinfo[motnum].rps_measured;
}

void motorcontrol_setrps(int motnum, float rps) {	// Desired vel in Centimeters/Second
	MotorInfo &mot = motinfo[motnum];

	if (sign(mot.rps_desired) != sign(rps)) { // if the sign of the rps changed
		pid_initstate(mot.pid); // reset the PID state
	}

	mot.rps_desired = rps; // update desired rps
}

void motorcontrol_setEnabled(bool new_enabled) {
	if (new_enabled) {
		for (int i=0; i<motor_count; i++)
			motinfo[i].prev_enc = enc_get(i);
	}

	enabled = new_enabled;
}

void motorcontrol_setDebug(bool new_debug) {
	debug = new_debug;
}

ISR(TIMOVFVEC) {
	if (!enabled)
		return;

	for (int motnum=0; motnum<motor_count; motnum++) { // for each motor
		MotorInfo &mot = motinfo[motnum]; // get its motor information

		uint16_t enc = enc_get(motnum); // read the amount the motor traveled since the last update
		int16_t diff = enc_diff(enc, mot.prev_enc); // compute the difference
		mot.rps_measured = diff/ticks_per_rotation*update_hz; // compute the rotations per second
		mot.prev_enc = enc; // save the encoder position

		float output = pid_update(mot.pid, pidcoefs, mot.rps_desired, mot.rps_measured, 1/update_hz); // update the PID loop
		output += sign(mot.rps_desired)*(fabs(mot.rps_desired)*mot.m + mot.b)/1024; // compute feedforward term

		if (output > 1) // enforce saturation on the output
			output = 1;
		else if (output < -1)
			output = -1;

		motor_setpwm(motnum, (int16_t)(output*motor_maxpwm)); // convert output to pwm, set it to the motor
	}

	if(debug)
		printf("MC %f %f %f %f %f\n",
			motinfo[0].rps_desired, motinfo[0].rps_measured, motinfo[0].pid.error_sum, motinfo[0].pid.error_last);
}

