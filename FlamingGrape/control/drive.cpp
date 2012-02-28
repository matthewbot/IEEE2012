#include "control/motorcontrol.h"
#include "control/drive.h"
#include "hw/enc.h"
#include "hw/motor.h"
#include "util.h"
#include <util/delay.h>
#include <stdio.h>

// Motion Constants:
static const float wheel_circumference = 16.3;
static const float wheelbase_radius = 9; // distance between wheels (needs cal)

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
		while (enc_diff(enc_get(mot), startenc) > distenc) { }
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

void drive_lturn_deg(float vel, float deg) {
	drive_dist(-vel, vel, degtorad(deg) * wheelbase_radius, MOTOR_RIGHT);
}

void drive_rturn(float vel) {
	drive(vel, -vel);
}

void drive_rturn_deg(float vel, float deg) {
	drive_dist(vel, -vel, degtorad(deg) * wheelbase_radius, MOTOR_LEFT);
}

void drive_steer(float steer, float vel) {
	drive(vel + steer, vel - steer);
}

void drive_wait_dist(float dist) {
	uint16_t leftenc = enc_get(MOTOR_LEFT);
	uint16_t rightenc = enc_get(MOTOR_RIGHT);
	
	while (true) {
		int16_t leftdiff = enc_diff(enc_get(MOTOR_LEFT), leftenc);
		int16_t rightdiff = enc_diff(enc_get(MOTOR_RIGHT), rightenc);
		float curdist = (leftdiff + rightdiff) * (wheel_circumference/2) / enc_per_rotation;
		
		if (dist > 0) {
			if (curdist >= dist)
				break;
		} else {
			if (curdist <= dist)
				break;
		}
	}
}
