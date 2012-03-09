#include "control/motorcontrol.h"
#include "control/traj.h"
#include "control/drive.h"
#include "hw/enc.h"
#include "hw/motor.h"
#include "util.h"
#include <util/delay.h>
#include <stdio.h>

// Motion Constants:
static const float wheel_circumference = 16.3;
static const float wheelbase_radius = 9; // distance between wheels (needs cal)

static float traj_amax_rps=100 / wheel_circumference; // 100 cm/s^2

void drive(float left, float right) {
	motorcontrol_setrps(MOTOR_LEFT, left / wheel_circumference);
	motorcontrol_setrps(MOTOR_RIGHT, right / wheel_circumference);
	motorcontrol_setEnabled(true);
}

void drive_dist(float leftvel, float rightvel, float ldist, float rdist, bool traj) {
	if (traj) {
		traj_setup(MOTOR_LEFT, ldist / wheel_circumference, 0, leftvel / wheel_circumference, ldist > 0 ? traj_amax_rps : -traj_amax_rps);
		traj_setup(MOTOR_RIGHT, rdist / wheel_circumference, 0, rightvel / wheel_circumference, rdist > 0 ? traj_amax_rps : -traj_amax_rps);
		traj_setEnabled(true);
		traj_wait();
	} else {
		float dist;
		Motor mot;
		if (fabs(ldist) > 0.1) {
			dist = ldist;
			mot = MOTOR_LEFT;
		} else {
			dist = rdist;
			mot = MOTOR_RIGHT;
		}
		
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

void drive_fdDist(float vel, float dist, bool traj) {
	drive_dist(vel, vel, dist, dist, traj);
}

void drive_bk(float vel) {
	drive(-vel, -vel);
}

void drive_bkDist(float vel, float dist, bool traj) {
	drive_dist(-vel, -vel, -dist, -dist, traj);
}

void drive_lturn(float vel) {
	drive(-vel, vel);
}

void drive_lturnDeg(float vel, float deg, bool traj) {
	float dist = degtorad(deg) * wheelbase_radius;
	drive_dist(-vel, vel, -dist, dist, traj);
}

void drive_rturn(float vel) {
	drive(vel, -vel);
}

void drive_rturnDeg(float vel, float deg, bool traj) {
	float dist = degtorad(deg) * wheelbase_radius;
	drive_dist(vel, -vel, dist, -dist, traj);
}

void drive_steer(float steer, float vel) {
	drive(vel + steer, vel - steer);
}

void drive_waitDist(float dist) {
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

void drive_setTrajAmax(float amax) {
	traj_amax_rps = amax / wheel_circumference;
}

float drive_getTrajAmax() {
	return traj_amax_rps * wheel_circumference;
}
