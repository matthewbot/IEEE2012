#define __STDC_LIMIT_MACROS

#include <stdio.h>

#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include <math.h>

#include "enc.h"
#include "motor.h"
#include "pid.h"

#include "motorcontrol.h"

static TC1_t &pidtim = TCF1;
#define TIMOVFVEC TCF1_OVF_vect

static const float ticks_per_rotation = 2500; // accurate
static const float update_hz = 50;
static const PIDCoefs pidcoefs = { 1, 1, .003*0, .01 };

static const float wheel_circumference = 16.1368; // Centimeters (may need to re-cal)

volatile static bool debug = false;

struct MotorInfo {
	PIDState pid; // state of PID loop
	volatile float rps_desired;
	volatile float rps_measured;
	volatile uint16_t prev_enc; // used to find motor velocity
	volatile bool enabled; // determines whether this motor is under PID control or not
};

static MotorInfo motinfo[2];

void motorcontrol_init() {
	pidtim.CTRLA = TC_CLKSEL_DIV64_gc; // timer runs at 1Mhz
	pidtim.INTCTRLA = TC_OVFINTLVL_MED_gc; // enable medium priority overflow interrupt
	pidtim.PER = 500000 / update_hz; // period computed, so overflow freq is update_hz
}

float motorcontrol_getvel(int motnum) {		// Returns rps not velocity
	return motinfo[motnum].rps_measured;
}

void motorcontrol_setvel(int motnum, float vel) {	// Desired vel in Centimeters/Second
	MotorInfo &mot = motinfo[motnum];
	
	float rps = vel/wheel_circumference;		// Gives revs/sec
	mot.rps_desired = rps; // update desired rps

	if (!mot.enabled) { // if the motor is currently disabled
		pid_initstate(mot.pid); // reset the PID state
		mot.prev_enc = enc_get(motnum); // start measuring encoder ticks from the current position
		mot.enabled = true; // the motor is now enabled
	}
}

void motorcontrol_setvel2(float vel0, float vel1) {
	MotorInfo &mot0 = motinfo[0];
	MotorInfo &mot1 = motinfo[1];

	float rps0 = vel0/wheel_circumference;
	float rps1 = vel1/wheel_circumference;
	mot0.rps_desired = rps0;
	mot1.rps_desired = rps1;

	if (!mot0.enabled) { // if the motor is currently disabled
		pid_initstate(mot0.pid); // reset the PID state
		mot0.prev_enc = enc_get(0); // start measuring encoder ticks from the current position
	}
	if (!mot1.enabled) { // if the motor is currently disabled
		pid_initstate(mot1.pid); // reset the PID state
		mot1.prev_enc = enc_get(1); // start measuring encoder ticks from the current position
	}
	mot0.enabled = true;
	mot1.enabled = true;
}

void motorcontrol_stop(int motnum) {		// for turning off an individual motor from setvel
	motorcontrol_setvel(motnum, 0);
}

void motorcontrol_stop() {			// for turning off both left and right motors from setvel
	motorcontrol_setvel2(0, 0);
}

void motorcontrol_disable(int motnum) {
	MotorInfo &mot = motinfo[motnum];

	mot.enabled = false;
	mot.rps_measured = 0;
	motor_setpwm(motnum, 0);
}

void motorcontrol_setDebug(bool new_debug) {
	debug = new_debug;
}

float sign(float in) {
	if (in > 0) {
		return 1.0;
	} else if (in < 0) {
		return -1.0;
	} else {
		return 0;
	}
}

ISR(TIMOVFVEC) {
	float d[2];
	for (int motnum=0; motnum<2; motnum++) { // for each motor
		MotorInfo &mot = motinfo[motnum]; // get its motor information

		if (!mot.enabled) // if its not enabled
			continue; // skip it

		uint16_t enc = enc_get(motnum); // read the amount the motor traveled since the last update
		int16_t difference = (int16_t)enc - (int16_t)mot.prev_enc;
		if (difference > INT16_MAX/2) {	// if encoder wrapped around
			difference -= INT16_MAX;
		} else if (difference < INT16_MIN/2) {
			difference += INT16_MAX;
		}
		mot.rps_measured = (difference)/ticks_per_rotation*update_hz; // compute the rotations per second
		mot.prev_enc = enc; // save the encoder position
		float output = pid_update(mot.pid, pidcoefs, mot.rps_desired, mot.rps_measured, 1/update_hz, &d[motnum]); // update the PID loop
		output += sign(mot.rps_desired)*((fabs(mot.rps_desired))/0.0128 + 577.125)/1024;
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

