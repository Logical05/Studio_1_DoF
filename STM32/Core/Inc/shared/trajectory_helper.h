/*
 * trajectory_helper.h
 *
 *  Created on: Jun 3, 2026
 *      Author: rajap
 */

#ifndef INC_SHARED_TRAJECTORY_HELPER_H_
#define INC_SHARED_TRAJECTORY_HELPER_H_

typedef struct {
	float pos;
	float vel;
	float acc;
} MotionState_t;

static inline MotionState_t Motion_IntegrateJerk(MotionState_t s, float jerk,
		float dt) {
	MotionState_t out;

	float dt2 = dt * dt;
	float dt3 = dt2 * dt;

	out.pos = s.pos + s.vel * dt + 0.5f * s.acc * dt2 + jerk * dt3 / 6.0f;
	out.vel = s.vel + s.acc * dt + 0.5f * jerk * dt2;
	out.acc = s.acc + jerk * dt;
	return out;
}

static inline MotionState_t Motion_IntegrateAccel(MotionState_t s, float accel,
		float dt) {
	MotionState_t out;

	out.pos = s.pos + s.vel * dt + 0.5f * accel * dt * dt;
	out.vel = s.vel + accel * dt;
	out.acc = accel;
	return out;
}

static inline MotionState_t Motion_IntegrateVelocity(MotionState_t s, float vel,
		float dt) {
	MotionState_t out;

	out.pos = s.pos + vel * dt;
	out.vel = vel;
	out.acc = 0.0f;
	return out;
}

#endif /* INC_SHARED_TRAJECTORY_HELPER_H_ */
