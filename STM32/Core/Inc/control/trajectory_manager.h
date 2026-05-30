/*
 * trajectory_manager.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_CONTROL_TRAJECTORY_MANAGER_H_
#define INC_CONTROL_TRAJECTORY_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	TRAJECTORY_DONE = 0, TRAJECTORY_WAIT, TRAJECTORY_RUN
} TrajectoryState_t;

typedef struct {
	const float *points;

	uint16_t num_points;
	uint16_t current;

	TrajectoryState_t state;

	float q0;
	float qf;

	float vmax;
	float amax;

	float T;
	float t;
} Trajectory_t;

extern Trajectory_t trajectory;

/* Lifecycle */
void Trajectory_Reset(void);

/* Runtime */
void Trajectory_Update(float dt);

void Trajectory_Start(const float *points, uint16_t count, float current_position,
		float vmax, float amax);

void Trajectory_Continue(void);

/* Status */
bool Trajectory_IsFinished(void);
TrajectoryState_t Trajectory_GetState(void);

/* Output */
float Trajectory_GetPosition(void);
float Trajectory_GetVelocity(void);
float Trajectory_GetAcceleration(void);

#endif /* INC_CONTROL_TRAJECTORY_MANAGER_H_ */
