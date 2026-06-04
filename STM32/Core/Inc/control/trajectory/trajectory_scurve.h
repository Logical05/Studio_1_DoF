/*
 * trajectory_scurve.h
 *
 *  Created on: Jun 3, 2026
 *      Author: rajap
 */

#ifndef INC_CONTROL_TRAJECTORY_SCURVE_H_
#define INC_CONTROL_TRAJECTORY_SCURVE_H_

#include "shared/trajectory_helper.h"

#include <stdint.h>

typedef enum {
    PROFILE_JERK_ONLY = 0,
    PROFILE_NO_CRUISE,
    PROFILE_FULL
} SCurveProfile_t;

typedef struct {

        float distance;

        float vmax;
        float amax;
        float jmax;

        SCurveProfile_t profile;

        float t1;
        float t2;
        float t3;
        float t4;
        float t5;
        float t6;
        float t7;

        float total_time;

        MotionState_t boundary[8];

} SCurveTrajectory_t;

void SCurve_Build(SCurveTrajectory_t *traj, float distance, float vmax, float amax,
                  float jmax);

MotionState_t SCurve_Evaluate(const SCurveTrajectory_t *traj, float t);

#endif /* INC_CONTROL_TRAJECTORY_SCURVE_H_ */
