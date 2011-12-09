#include <util/delay.h>

#include "control/motorcontrol.h"

#include "control/drive.h"

// Motion Constants:
static const float wheel_circumference = 16.1368; // Centimeters (may need to re-cal)
static const float turn_360_const = 5.7; // Time in Seconds it takes to turn 360 Degrees
static const float pivot_360_const = 11.5; // NEED TO CALIBRATE.
// Motor ports:
static const uint16_t l_motor = 0;
static const uint16_t r_motor = 1;
// Motor polarities:
static const int16_t l_polarity = 1;
static const int16_t r_polarity = 1;

void drive(float left, float right) {
	motorcontrol_setrps(MOTOR_LEFT, left / wheel_circumference);
	motorcontrol_setrps(MOTOR_RIGHT, right / wheel_circumference);
	motorcontrol_setEnabled(true);
}

void drive_stop() {
	motorcontrol_setrps(MOTOR_LEFT, 0);
	motorcontrol_setrps(MOTOR_RIGHT, 0);
}

void drive_fwd(float dist, float vel) {	// Drive forward 'dist' desired in Centimeters, 'vel' desired in Centimeters/Second
	drive(l_polarity*vel, r_polarity*vel);	// Turns on motors at their velocities
	uint16_t time = (int)((dist/vel)*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);		// wait 'time' number of Seconds
	}
	drive_stop();	// Turns off motors
}

void drive_bck(float dist, float vel) {
	drive(-l_polarity*vel, -r_polarity*vel);	// Turns on motors at their velocities
	uint16_t time = (int)((dist/vel)*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);		// wait 'time' number of Seconds
	}
	drive_stop();	// Turns off motors
}

void drive_l_turn(float degrees, float vel) {
	drive(-l_polarity*vel, r_polarity*vel);
	uint16_t time = (int)((degrees/360)*turn_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);	// wait for number of seconds it takes to turn degrees portion of known 360 constant
	}
	drive_stop();
}

void drive_r_turn(float degrees, float vel) {
	drive(l_polarity*vel, -r_polarity*vel);
	uint16_t time = (int)((degrees/360)*turn_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	drive_stop();
}

void drive_l_piv_bck(float degrees, float vel) {		// Pivots backwards facing left (about right wheel)
	drive(-l_polarity*vel, 0);
	uint16_t time = (int)((degrees/360)*pivot_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	drive_stop();
}

void drive_l_piv_fwd(float degrees, float vel) {		// Pivots forwards facing left (about left wheel)
	drive(0, r_polarity*vel);
	uint16_t time = (int)((degrees/360)*pivot_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	drive_stop();
}

void drive_r_piv_bck(float degrees, float vel) {		// Pivots backwards facing right (about left wheel)
	drive(0, -r_polarity*vel);
	uint16_t time = (int)((degrees/360)*pivot_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	drive_stop();
}

void drive_r_piv_fwd(float degrees, float vel) {		// Pivots forwards facing right (about right wheel)
	drive(l_polarity*vel, 0);
	uint16_t time = (int)((degrees/360)*pivot_360_const*1000);
	for (int i = 0; i < time; i++) {
		_delay_ms(1);
	}
	drive_stop();
}

