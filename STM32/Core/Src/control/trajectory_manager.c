/*
 * trajectory_manager.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "control/trajectory_manager.h"
#include "shared/robot_math.h"

#include "math.h"
#include "stm32g4xx_hal.h"

Trajectory_t trajectory = {0};

/* ============================================================
 * Quintic trajectory equations
 * ============================================================ */

static float quintic_pos(float t, float T, float q0, float qf)
{
	if (T <= 0.0f)
		return qf;

	float tau = clampf(t / T, 0.0f, 1.0f);

	float t2 = tau * tau;
	float t3 = t2 * tau;
	float t4 = t3 * tau;
	float t5 = t4 * tau;

	return q0 + (qf - q0) * (10.0f * t3 - 15.0f * t4 + 6.0f * t5);
}

static float quintic_vel(float t, float T, float q0, float qf)
{
	if (T <= 0.0f)
		return 0.0f;

	float tau = clampf(t / T, 0.0f, 1.0f);

	float t2 = tau * tau;
	float t3 = t2 * tau;
	float t4 = t3 * tau;

	return (qf - q0) * (30.0f * t2 - 60.0f * t3 + 30.0f * t4) / T;
}

static float quintic_acc(float t, float T, float q0, float qf)
{
	if (T <= 0.0f)
		return 0.0f;

	float tau = clampf(t / T, 0.0f, 1.0f);

	float t2 = tau * tau;
	float t3 = t2 * tau;

	return (qf - q0) * (60.0f * tau - 180.0f * t2 + 120.0f * t3) / (T * T);
}

/* ============================================================
 * Time calculation
 * ============================================================ */

static float calc_time(float q0, float qf, float vmax, float amax)
{
	float dq = fabsf_fast(qf - q0);

	if (dq < 0.0001f)
	{
		return 0.01f;
	}

	if (vmax <= 0.0f)
	{
		vmax = 0.001f;
	}

	if (amax <= 0.0f)
	{
		amax = 0.001f;
	}

	float Tv = 1.875f * dq / vmax;
	float Ta = sqrtf(5.7735f * dq / amax);

	float T = (Tv > Ta) ? Tv : Ta;

	if (T < 0.05f)
	{
		T = 0.05f;
	}

	return T;
}

/* ============================================================
 * Internal
 * ============================================================ */

static void load_segment(void)
{
	trajectory.qf = trajectory.points[trajectory.current];

	trajectory.T = calc_time(trajectory.q0, trajectory.qf, trajectory.vmax,
							 trajectory.amax);

	trajectory.t = 0.0f;
}

/* ============================================================
 * Public API
 * ============================================================ */

void Trajectory_Reset(void)
{
	__disable_irq();

	trajectory.points = 0;

	trajectory.num_points = 0;
	trajectory.current = 0;

	trajectory.state = TRAJECTORY_DONE;

	trajectory.q0 = 0.0f;
	trajectory.qf = 0.0f;

	trajectory.vmax = 0.0f;
	trajectory.amax = 0.0f;

	trajectory.T = 0.0f;
	trajectory.t = 0.0f;

	__enable_irq();
}

void Trajectory_Start(const float *points, uint16_t count, float current_position,
					  float vmax, float amax)
{
	if (points == 0 || count == 0)
	{
		Trajectory_Reset();
		return;
	}

	__disable_irq();

	trajectory.points = points;

	trajectory.num_points = count;
	trajectory.current = 0;

	trajectory.q0 = current_position;

	trajectory.vmax = vmax;
	trajectory.amax = amax;

	trajectory.state = TRAJECTORY_RUN;

	load_segment();

	__enable_irq();
}

void Trajectory_Update(float dt)
{
	if (trajectory.state != TRAJECTORY_RUN)
	{
		return;
	}

	trajectory.t += dt;

	if (trajectory.t >= trajectory.T)
	{
		trajectory.t = trajectory.T;

		trajectory.q0 = trajectory.qf;

		trajectory.state = TRAJECTORY_WAIT;
	}
}

void Trajectory_Continue(void)
{
	__disable_irq();

	if (trajectory.state != TRAJECTORY_WAIT)
	{
		__enable_irq();
		return;
	}

	trajectory.current++;

	if (trajectory.current >= trajectory.num_points)
	{
		trajectory.current = trajectory.num_points - 1;

		trajectory.state = TRAJECTORY_DONE;

		__enable_irq();
		return;
	}

	load_segment();

	trajectory.state = TRAJECTORY_RUN;

	__enable_irq();
}

bool Trajectory_IsFinished(void)
{
	return trajectory.state == TRAJECTORY_DONE;
}

TrajectoryState_t Trajectory_GetState(void)
{
	return trajectory.state;
}

float Trajectory_GetPosition(void)
{
	return quintic_pos(trajectory.t, trajectory.T, trajectory.q0, trajectory.qf);
}

float Trajectory_GetVelocity(void)
{
	return quintic_vel(trajectory.t, trajectory.T, trajectory.q0, trajectory.qf);
}

float Trajectory_GetAcceleration(void)
{
	return quintic_acc(trajectory.t, trajectory.T, trajectory.q0, trajectory.qf);
}
