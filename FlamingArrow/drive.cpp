#include <util/delay.h>

#include "motorcontrol.h"

// Motion Constants:
static const float turn_360_const = 10; // NEED TO CALIBRATE. Time in Seconds it takes to turn 360 Degrees
static const float pivot_360_const = 20; // NEED TO CALIBRATE.
// Motor ports:
static const uint16_t l_motor = 0;
static const uint16_t r_motor = 1;
// Motor polarities:
static const uint16_t l_polarity = 1;
static const uint16_t r_polarity = 1;

void drive_fwd(float dist, float vel) {	// Drive forward 'dist' desired in Centimeters, 'vel' desired in Centimeters/Second
	motorcontrol_setvel(l_motor, l_polarity*vel);	// Turns on motors at their velocities
	motorcontrol_setvel(r_motor, r_polarity*vel);
	_delay_ms(time*1000);		// wait 'time' number of Seconds
	motorcontrol_stop();	// Turns off motors
}

void drive_bck(float dist, float vel) {
	motorcontrol_setvel(l_motor, -l_polarity*vel);	// Turns on motors at their velocities
	motorcontrol_setvel(r_motor, -r_polarity*vel);
	_delay_ms(time*1000);		// wait 'time' number of Seconds
	motorcontrol_stop();	// Turns off motors
}

void drive_l_turn(float degrees, float vel) {
	motorcontrol_setvel(l_motor, -l_polarity*vel);
	motorcontrol_setvel(r_motor, r_polarity*vel);
	_delay_ms((degrees/360)*turn_360_const*1000);	// wait for number of seconds it takes to turn degrees portion of known 360 constant
	motorcontrol_stop();
}

void drive_r_turn(float degrees, float vel) {
	motorcontrol_setvel(l_motor, l_polarity*vel);
	motorcontrol_setvel(r_motor, -r_polarity*vel);
	_delay_ms((degrees/360)*turn_360_const*1000);
	motorcontrol_stop();
}

void drive_l_piv_bck(float degrees, float vel) {		// Pivots backwards facing left (about right wheel)
	motorcontrol_setvel(l_motor, -l_polarity*vel);
	_delay_ms((degrees/360)*pivot_360_const*1000);
	motorcontrol_stop();
}

void drive_l_piv_fwd(float degrees, float vel) {		// Pivots forwards facing left (about left wheel)
	motorcontrol_setvel(r_motor, r_polarity*vel);
	_delay_ms((degrees/360)*pivot_360_const*1000);
	motorcontrol_stop();
}

void drive_r_piv_bck(float degrees, float vel) {		// Pivots backwards facing right (about left wheel)
	motorcontrol_setvel(r_motor, -r_polarity*vel);
	_delay_ms((degrees/360)*pivot_360_const*1000);
	motorcontrol_stop();
}

void drive_r_piv_fwd(float degrees, float vel) {		// Pivots forwards facing right (about right wheel)
	motorcontrol_setvel(l_motor, l_polarity*vel);
	_delay_ms((degrees/360)*pivot_360_const*1000);
	motorcontrol_stop();
}
