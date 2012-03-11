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
static const uint16_t debounce = 300;	// ms needed for hard cstop to stop bouncing before next move

static float traj_amax_rps=150 / wheel_circumference; // 100 cm/s^2

void drive(float lvel, float rvel, DriveMode dm) {
	if (dm == DM_TRAJ) {
		traj_goVel(lvel / wheel_circumference, traj_amax_rps,
		           rvel / wheel_circumference, traj_amax_rps);
	} else {
		traj_stop();
		motorcontrol_setRPS(MOTOR_LEFT,  lvel / wheel_circumference); // floor it!
		motorcontrol_setRPS(MOTOR_RIGHT, rvel / wheel_circumference);
		motorcontrol_setEnabled(true);
	}
}

void drive_dist(float lvel, float rvel, float ldist, float rdist, DriveMode dm) {
	if (dm == DM_TRAJ) {
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
		
		drive(lvel, rvel, DM_BANG);
		if (dist > 0) {
			while (enc_diff(enc_get(mot), startenc) < distenc) { }
		} else {
			while (enc_diff(enc_get(mot), startenc) > distenc) { }
		}
		drive_stop();
	}
}

void drive_stop(DriveMode dm) {
	drive(0, 0, dm);
	if (dm == DM_TRAJ)
		traj_wait();
}

void drive_cStop() {
	drive_stop();
	_delay_ms(debounce);
}

void drive_off() {
	traj_stop();
	motorcontrol_setEnabled(false);
}

void drive_fd(float vel, DriveMode dm) {
	drive(vel, vel, dm);
}

void drive_fdDist(float vel, float dist, DriveMode dm) {
	drive_dist(vel, vel, dist, dist, dm);
}

void drive_bk(float vel, DriveMode dm) {
	drive(-vel, -vel, dm);
}

void drive_bkDist(float vel, float dist, DriveMode dm) {
	drive_dist(-vel, -vel, -dist, -dist, dm);
}

void drive_lturn(float vel, DriveMode dm) {
	drive(-vel, vel, dm);
}

void drive_lturnDeg(float vel, float deg, DriveMode dm) {
	float dist = degtorad(deg) * wheelbase_radius;
	drive_dist(-vel, vel, -dist, dist, dm);
}

void drive_rturn(float vel, DriveMode dm) {
	drive(vel, -vel, dm);
}

void drive_rturnDeg(float vel, float deg, DriveMode dm) {
	float dist = degtorad(deg) * wheelbase_radius;
	drive_dist(vel, -vel, dist, -dist, dm);
}

void drive_steer(float steer, float vel) {
	drive(vel + steer, vel - steer, DM_BANG);
}

void drive_waitDist(float dist) {
	DriveDist dd;
	drive_initDist(dd);
	
	while (true) {
		float curdist = drive_getDist(dd);
		
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

void drive_initDist(DriveDist &dist) {
	dist.leftenc = enc_get(MOTOR_LEFT);
	dist.rightenc = enc_get(MOTOR_RIGHT);	
}

float drive_getDist(const DriveDist &dist) {
	int16_t leftdiff = enc_diff(enc_get(MOTOR_LEFT), dist.leftenc);
	int16_t rightdiff = enc_diff(enc_get(MOTOR_RIGHT), dist.rightenc);
	return (leftdiff + rightdiff) * ((wheel_circumference/2) / enc_per_rotation);
}
