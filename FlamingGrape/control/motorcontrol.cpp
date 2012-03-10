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
	volatile float RPS_desired;
	volatile float RPS_measured;
	volatile uint16_t prev_enc; // used to find motor velocity
};

static MotorInfo motinfo[motorcontrol_count];

void motorcontrol_init() {
	motinfo[0].m = 43.838  / motor_maxPWM;
	motinfo[0].b = 488.072 / motor_maxPWM;
	motinfo[1].m = 43.683  / motor_maxPWM;
	motinfo[1].b = 472.567 / motor_maxPWM;
}

float motorcontrol_getRPS(int motnum) {
	return motinfo[motnum].RPS_measured;
}

float motorcontrol_getRPSDesired(int motnum) {
	return motinfo[motnum].RPS_desired;
}

void motorcontrol_setRPS(int motnum, float RPS) {
	MotorInfo &mot = motinfo[motnum];
	
	if (sign(mot.RPS_desired) != sign(RPS))
		mot.pid.sum = 0;
	
	mot.RPS_desired = RPS;
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
		for (int i=0; i<motorcontrol_count; i++)
			motor_setPWM(i, 0);
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
	if (!enabled) {
		debug_setLED(YELLOW_LED, false);
		return;
	}

	bool led=false;

	for (int motnum=0; motnum<motorcontrol_count; motnum++) { // for each motor
		MotorInfo &mot = motinfo[motnum]; // get its motor information

		uint16_t enc = enc_get(motnum); // read the amount the motor traveled since the last update
		int16_t diff = enc_diff(enc, mot.prev_enc); // compute the difference
		mot.RPS_measured = diff/enc_per_rotation*TICK_HZ; // compute the rotations per second
		mot.prev_enc = enc; // save the encoder position

		PIDDebug piddebug;
		float error = mot.RPS_desired - mot.RPS_measured; // compute error
		float out = pid_update(mot.pid, pidgains, error, 1.0/TICK_HZ, &piddebug); // update pid
		out += sign(mot.RPS_desired)*(mot.m*fabs(mot.RPS_desired) + mot.b); // compute feedforward
		
		if (debug && motnum == 0)
			pid_printDebug(out, error, piddebug);
			
		if (out > 1) // enforce saturation
			out = 1;
		else if (out < -1)
			out = -1;
		
		motor_setPWM(motnum, (int16_t)(out*motor_maxPWM)); // convert output to PWM, set it to the motor
		
		if (error > .5)
			led = true;
	}
	
	debug_setLED(YELLOW_LED, led);
}

