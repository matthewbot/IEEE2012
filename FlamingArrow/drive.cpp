#include <util/delay.h>

#include "motorcontrol.h"
#include "drive.h"

// Motion Constants:
static const float turn_360_const = 10; // NEED TO CALIBRATE. Time in Seconds it takes to turn 360 Degrees
static const float pivot_360_const = 20; // NEED TO CALIBRATE.
// Motor ports:
static const uint16_t l_motor = 0;
static const uint16_t r_motor = 1;
// Motor polarities:
static const int16_t l_polarity = 1;
static const int16_t r_polarity = 1;

void drive_fwd(float dist, float vel) {	// Drive forward 'dist' desired in Centimeters, 'vel' desired in Centimeters/Second
	motorcontrol_setvel2(l_polarity*vel, r_polarity*vel);	// Turns on motors at their velocities
	uint16_t time = (int)((dist/vel)*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);		// wait 'time' number of Seconds
	}
	motorcontrol_stop();	// Turns off motors
}

void drive_bck(float dist, float vel) {
	motorcontrol_setvel2(-1*l_polarity*vel, -1*r_polarity*vel);	// Turns on motors at their velocities
	uint16_t time = (int)((dist/vel)*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);		// wait 'time' number of Seconds
	}
	motorcontrol_stop();	// Turns off motors
}

void drive_l_turn(float degrees, float vel) {
	motorcontrol_setvel2(-1*l_polarity*vel, r_polarity*vel);
	uint16_t time = (int)((degrees/360)*turn_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);	// wait for number of seconds it takes to turn degrees portion of known 360 constant
	}
	motorcontrol_stop();
}

void drive_r_turn(float degrees, float vel) {
	motorcontrol_setvel2(l_polarity*vel, -1*r_polarity*vel);
	uint16_t time = (int)((degrees/360)*turn_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	motorcontrol_stop();
}

void drive_l_piv_bck(float degrees, float vel) {		// Pivots backwards facing left (about right wheel)
	motorcontrol_setvel(l_motor, -1*l_polarity*vel);
	uint16_t time = (int)((degrees/360)*pivot_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	motorcontrol_stop();
}

void drive_l_piv_fwd(float degrees, float vel) {		// Pivots forwards facing left (about left wheel)
	motorcontrol_setvel(r_motor, r_polarity*vel);
	uint16_t time = (int)((degrees/360)*pivot_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	motorcontrol_stop();
}

void drive_r_piv_bck(float degrees, float vel) {		// Pivots backwards facing right (about left wheel)
	motorcontrol_setvel(r_motor, -1*r_polarity*vel);
	uint16_t time = (int)((degrees/360)*pivot_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	motorcontrol_stop();
}

void drive_r_piv_fwd(float degrees, float vel) {		// Pivots forwards facing right (about right wheel)
	motorcontrol_setvel(l_motor, l_polarity*vel);
	uint16_t time = (int)((degrees/360)*pivot_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	motorcontrol_stop();
}
