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

void drive(float lvel, float rvel, bool traj) {
	if (traj) {
		traj_goVel(lvel / wheel_circumference, traj_amax_rps,
		           rvel / wheel_circumference, traj_amax_rps);
	} else {
		traj_stop();
		motorcontrol_setrps(MOTOR_LEFT,  lvel / wheel_circumference); // floor it!
		motorcontrol_setrps(MOTOR_RIGHT, rvel / wheel_circumference);
		motorcontrol_setEnabled(true);
	}
}

void drive_dist(float lvel, float rvel, float ldist, float rdist, bool traj) {
	if (traj) {
		traj_goDist(ldist / wheel_circumference, 0, lvel / wheel_circumference, traj_amax_rps,
		            rdist / wheel_circumference, 0, rvel / wheel_circumference, traj_amax_rps);
		traj_wait();
	} else {
		traj_stop();
		
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
		
		drive(lvel, rvel, false);
		if (dist > 0) {
			while (enc_diff(enc_get(mot), startenc) < distenc) { }
		} else {
			while (enc_diff(enc_get(mot), startenc) > distenc) { }
		}
		drive_stop(false);
	}
}

void drive_stop(bool traj) {
	drive(0, 0, traj);
}

void drive_off() {
	traj_stop();
	motorcontrol_setEnabled(false);
}

void drive_fd(float vel, bool traj) {
	drive(vel, vel, traj);
}

void drive_fd_dist(float vel, float dist, bool traj) {
	drive_dist(vel, vel, dist, dist, traj);
}

void drive_bk(float vel, bool traj) {
	drive(-vel, -vel, traj);
}

void drive_bk_dist(float vel, float dist, bool traj) {
	drive_dist(-vel, -vel, -dist, -dist, traj);
}

void drive_lturn(float vel, bool traj) {
	drive(-vel, vel, traj);
}

void drive_lturn_deg(float vel, float deg, bool traj) {
	float dist = degtorad(deg) * wheelbase_radius;
	drive_dist(-vel, vel, -dist, dist, traj);
}

void drive_rturn(float vel, bool traj) {
	drive(vel, -vel, traj);
}

void drive_rturn_deg(float vel, float deg, bool traj) {
	float dist = degtorad(deg) * wheelbase_radius;
	drive_dist(vel, -vel, dist, -dist, traj);
}

void drive_steer(float steer, float vel) {
	drive(vel + steer, vel - steer, false);
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

void drive_setTrajAmax(float amax) {
	traj_amax_rps = amax / wheel_circumference;
}

float drive_getTrajAmax() {
	return traj_amax_rps * wheel_circumference;
}
