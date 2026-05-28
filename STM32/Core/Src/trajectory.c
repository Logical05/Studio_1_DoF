/*
 * trajectory.c
 *
 * Minimum Jerk Trajectory
 */

#include "trajectory.h"

#include <math.h>
#include "useful.h"

/*
 * Fastest possible time
 */
static float calc_time(float q0, float qf, float vmax, float amax) {
	float dq = fabsf(qf - q0);

	float Tv = 1.875f * dq / vmax;
	float Ta = sqrtf(5.7735f * dq / amax);
//	float T = fmaxf(Tv, Ta);
	float T = Ta;

	/*
	 * Minimum practical time
	 */

	if (T < 0.8f) T = 0.8f;

	return T;
}

/* ============================================================
 * Single Segment
 * ============================================================ */

float MJT_Pos(float t, float T, float q0, float qf) {
	float tau = clamp(t / T, 0.0f, 1.0f);

	float t2 = tau * tau;
	float t3 = t2 * tau;
	float t4 = t3 * tau;
	float t5 = t4 * tau;

	return q0 + (qf - q0) * (10.0f * t3 - 15.0f * t4 + 6.0f * t5);
}

float MJT_Vel(float t, float T, float q0, float qf) {
	float tau = clamp(t / T, 0.0f, 1.0f);

	float t2 = tau * tau;
	float t3 = t2 * tau;
	float t4 = t3 * tau;

	return (qf - q0) * (30.0f * t2 - 60.0f * t3 + 30.0f * t4) / T;
}

float MJT_Acc(float t, float T, float q0, float qf) {
	float tau = clamp(t / T, 0.0f, 1.0f);

	float t2 = tau * tau;
	float t3 = t2 * tau;

	return (qf - q0) * (60.0f * tau - 180.0f * t2 + 120.0f * t3) / (T * T);
}

/* ============================================================
 * Internal
 * ============================================================ */

static void load_segment(MJT_Trajectory *traj) {
	traj->qf = traj->points[traj->current];

	traj->T = calc_time(traj->q0, traj->qf, traj->vmax, traj->amax);

	traj->t = 0.0f;
}

/* ============================================================
 * Multi Segment
 * ============================================================ */

void MJT_Reset(MJT_Trajectory *traj) {
	__disable_irq();
	traj->points = NULL;
	traj->state = MJT_IDLE;

	traj->num_points = 0;
	traj->current = 0;

	traj->q0 = 0.0f;
	traj->qf = 0.0f;

	traj->vmax = 0.0f;
	traj->amax = 0.0f;

	traj->T = 0.0f;
	traj->t = 0.0f;
	__enable_irq();
}

void MJT_Goal(MJT_Trajectory *traj, const float *points, int num_points,
		float q_start, float vmax, float amax) {
	__disable_irq();
	traj->points = points;

	traj->num_points = num_points;

	traj->state = MJT_RUN;

	traj->current = 0;

	traj->q0 = q_start;

	traj->vmax = vmax;
	traj->amax = amax;

	load_segment(traj);
	__enable_irq();
}

void MJT_Update(MJT_Trajectory *traj, float dt) {
	if (traj->state != MJT_RUN) return;

	traj->t += dt;

	if (traj->t >= traj->T) {
		traj->t = traj->T;

		traj->q0 = traj->qf;

		traj->state = MJT_WAIT;
	}
}

void MJT_Continue(MJT_Trajectory *traj) {
	__disable_irq();
	if (traj->state != MJT_WAIT) {
		__enable_irq();
		return;
	}

	traj->current++;

	if (traj->current >= traj->num_points) {
		traj->current = traj->num_points - 1;

		traj->state = MJT_DONE;

		return;
	}

	load_segment(traj);

	traj->state = MJT_RUN;
	__enable_irq();
}

float MJT_get_Pos(const MJT_Trajectory *traj) {
	return MJT_Pos(traj->t, traj->T, traj->q0, traj->qf);
}

float MJT_get_Vel(const MJT_Trajectory *traj) {
	return MJT_Vel(traj->t, traj->T, traj->q0, traj->qf);
}

float MJT_get_Acc(const MJT_Trajectory *traj) {
	return MJT_Acc(traj->t, traj->T, traj->q0, traj->qf);
}

bool MJT_is_Finished(const MJT_Trajectory *traj) {
	return traj->state == MJT_DONE;
}
