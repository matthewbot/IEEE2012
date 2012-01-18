#include "control/motorcontrol.h"
#include "control/drive.h"
#include "hw/enc.h"
#include "hw/motor.h"
#include "util.h"

#include <util/delay.h>

// Motion Constants:
static const float wheel_circumference = 16.3; // Centimeters (may need to re-cal)
static const float wheelbase_radius = 11; // distance between wheels (needs cal)

void drive(float left, float right) {
	motorcontrol_setrps(MOTOR_LEFT, left / wheel_circumference);
	motorcontrol_setrps(MOTOR_RIGHT, right / wheel_circumference);
	motorcontrol_setEnabled(true);
}

void drive_dist(float leftvel, float rightvel, float dist, int mot) {
	int16_t distenc = (int16_t)(dist / wheel_circumference * enc_per_rotation);
	uint16_t startenc = enc_get(mot);
	drive(leftvel, rightvel);
	
	if (dist > 0) {
		while (enc_diff(enc_get(mot), startenc) < distenc) { }
	} else {
		while (enc_diff(enc_get(mot), startenc) > -distenc) { }
	}
	
	drive_stop();
}

void drive_stop() {
	drive(0, 0);
}

void drive_off() {
	motorcontrol_setEnabled(false);
}

void drive_fd(float vel) {
	drive(vel, vel);
}

void drive_fd_dist(float vel, float dist) {
	drive_dist(vel, vel, dist, MOTOR_LEFT);
}

void drive_bk(float vel) {
	drive(-vel, -vel);
}

void drive_bk_dist(float vel, float dist) {
	drive_dist(-vel, -vel, -dist, MOTOR_LEFT);
}

void drive_lturn(float vel) {
	drive(-vel, vel);
}

void drive_lturn_deg(float deg, float vel) {
	drive_dist(-vel, vel, degtorad(deg) * wheelbase_radius, MOTOR_RIGHT);
}

void drive_rturn(float vel) {
	drive(vel, -vel);
}

void drive_rturn_deg(float deg, float vel) {
	drive_dist(vel, -vel, degtorad(deg) * wheelbase_radius, MOTOR_LEFT);
}

void drive_steer(float steer, float vel) {
	float s = 1-fabsf(steer);
	vel *= .8*s*s + .2;
	
	float lvel, rvel;
	if (steer > 0) {
		lvel = vel;
		rvel = vel * (1 - 2*steer);
	} else {
		lvel = vel * (1 - 2*steer);
		rvel = vel;
	}
	
	drive(lvel, rvel);
}
