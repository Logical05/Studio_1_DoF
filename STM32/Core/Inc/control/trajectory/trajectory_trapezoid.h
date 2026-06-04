/*
 * trajectory_trapezoid.h
 *
 *  Created on: Jun 3, 2026
 *      Author: rajap
 */

#ifndef INC_CONTROL_TRAJECTORY_TRAPEZOID_H_
#define INC_CONTROL_TRAJECTORY_TRAPEZOID_H_

#include "shared/trajectory_helper.h"

#include <stdint.h>

typedef struct {

        float distance;

        float vmax;
        float amax;

        float ta;
        float tc;
        float td;

        float total_time;

        MotionState_t boundary[4];

} TrapezoidTrajectory_t;

void Trapezoid_Build(TrapezoidTrajectory_t *traj, float distance, float vmax,
                     float amax);

MotionState_t Trapezoid_Evaluate(const TrapezoidTrajectory_t *traj, float t);

#endif /* INC_CONTROL_TRAJECTORY_TRAPEZOID_H_ */
