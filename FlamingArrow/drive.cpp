#include <util/delay.h>

#include "motorcontrol.h"

// Motion Constants
static const float turn_360_const = 100; // NEED TO CALIBRATE. Time in Seconds it takes to turn 360 Degrees
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

void l_turn(float degrees, float vel) {
	motorcontrol_setvel(l_motor, -l_polarity*vel);
	motorcontrol_setvel(r_motor, r_polarity*vel);
	_delay_ms((degrees/360)*turn_360_const*1000);	// wait for number of seconds it takes to turn degrees portion of known 360 constant
	motorcontrol_stop();
}

void r_turn(float degrees, float vel) {
	motorcontrol_setvel(l_motor, l_polarity*vel);
	motorcontrol_setvel(r_motor, -r_polarity*vel);
	_delay_ms((degrees/360)*turn_360_const*1000);
	motorcontrol_stop();
}
