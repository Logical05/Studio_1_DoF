/*
 * trajectory.h
 *
 * Minimum Jerk Trajectory
 *
 * Auto calculate segment time from vmax
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

#include <stdbool.h>

typedef enum {
	MJT_IDLE, MJT_RUN, MJT_WAIT, MJT_DONE
} MJT_State;

/*
 * Trajectory Object
 */
typedef struct {

	const float *points;

	int num_points;

	MJT_State state;

	int current;

	float q0;
	float qf;

	float vmax;
	float amax;

	float T;

	float t;

} MJT_Trajectory;

/* ============================================================
 * Single Segment
 * ============================================================ */

float MJT_Pos(float t, float T, float q0, float qf);

float MJT_Vel(float t, float T, float q0, float qf);

float MJT_Acc(float t, float T, float q0, float qf);

/* ============================================================
 * Multi Segment
 * ============================================================ */

void MJT_Reset(MJT_Trajectory *traj);

void MJT_Goal(MJT_Trajectory *traj, const float *points, int num_points,
		float q_start, float vmax, float amax);

void MJT_Update(MJT_Trajectory *traj, float dt);

void MJT_Continue(MJT_Trajectory *traj);

float MJT_get_Pos(const MJT_Trajectory *traj);

float MJT_get_Vel(const MJT_Trajectory *traj);

float MJT_get_Acc(const MJT_Trajectory *traj);

bool MJT_is_Finished(const MJT_Trajectory *traj);

#endif
