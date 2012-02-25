#include "control/motorcontrol.h"
#include "control/pid.h"
#include "debug/debug.h"
#include "hw/enc.h"
#include "hw/motor.h"
#include "hw/tick.h"
#include "util.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdint.h>

static PIDGains pidgains = { 4, 1, .005, .5 };

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

static MotorInfo motinfo[motorcontrol_count];

void motorcontrol_init() {
	motinfo[0].m = 51.307   / motor_maxpwm;
	motinfo[0].b = 473.6371 / motor_maxpwm;
	motinfo[1].m = 48.755   / motor_maxpwm;
	motinfo[1].b = 476.1585 / motor_maxpwm;
}

float motorcontrol_getrps(int motnum) {
	return motinfo[motnum].rps_measured;
}

void motorcontrol_setrps(int motnum, float rps) {
	MotorInfo &mot = motinfo[motnum];

	if (sign(mot.rps_desired) != sign(rps) || fabsf(rps) < 0.1 || fabsf(mot.rps_desired) < 0.1)
		mot.pid.sum = 0;
	
	mot.rps_desired = rps;
}

void motorcontrol_setEnabled(bool new_enabled) {
	if (enabled == new_enabled)
		return;
	
	if (new_enabled) {
		for (int i=0; i<motorcontrol_count; i++)
			motinfo[i].prev_enc = enc_get(i);
		enabled = true;
	} else {
		enabled = false;
		motor_allOff();
	}
}

void motorcontrol_setGains(const PIDGains &newpidgains) {
	tick_suspend();
	pidgains = newpidgains;
	tick_resume();
}

PIDGains motorcontrol_getGains() {
	return pidgains;
}

void motorcontrol_setDebug(bool newdebug) {
	debug = newdebug;
}

void motorcontrol_tick() {
	if (!enabled)
		return;

	for (int motnum=0; motnum<motorcontrol_count; motnum++) { // for each motor
		MotorInfo &mot = motinfo[motnum]; // get its motor information

		uint16_t enc = enc_get(motnum); // read the amount the motor traveled since the last update
		int16_t diff = enc_diff(enc, mot.prev_enc); // compute the difference
		mot.rps_measured = diff/enc_per_rotation*TICK_HZ; // compute the rotations per second
		mot.prev_enc = enc; // save the encoder position

		PIDDebug piddebug;
		float error = mot.rps_desired - mot.rps_measured; // compute error
		float out = pid_update(mot.pid, pidgains, error, 1.0/TICK_HZ, &piddebug); // update pid
		out += sign(mot.rps_desired)*(mot.m*fabs(mot.rps_desired) + mot.b); // compute feedforward
		
		if (debug && motnum == 0)
			pid_printDebug(out, error, piddebug);
			
		if (out > 1) // enforce saturation
			out = 1;
		else if (out < -1)
			out = -1;
		
		motor_setpwm(motnum, (int16_t)(out*motor_maxpwm)); // convert output to pwm, set it to the motor
	}
}

