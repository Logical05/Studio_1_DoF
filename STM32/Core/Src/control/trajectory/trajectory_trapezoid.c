/*
 * trajectory_trapezoid.c
 *
 *  Created on: Jun 3, 2026
 *      Author: rajap
 */

#include "control/trajectory/trajectory_trapezoid.h"

#include <math.h>

void Trapezoid_Build(TrapezoidTrajectory_t *traj, float distance, float vmax,
                     float amax) {
    float ta = vmax / amax;

    float da = 0.5f * amax * ta * ta;

    float cruise = 0.0f;

    if (distance > 2.0f * da) {
        cruise = (distance - 2.0f * da) / vmax;
    } else {
        ta   = sqrtf(distance / amax);
        vmax = amax * ta;
    }

    traj->distance = distance;

    traj->vmax = vmax;
    traj->amax = amax;

    traj->ta = ta;
    traj->tc = ta + cruise;
    traj->td = traj->tc + ta;

    traj->total_time = traj->td;

    MotionState_t s = {0};

    traj->boundary[0] = s;

    s = Motion_IntegrateAccel(s, +amax, ta);

    traj->boundary[1] = s;

    s = Motion_IntegrateVelocity(s, vmax, cruise);

    traj->boundary[2] = s;

    s = Motion_IntegrateAccel(s, -amax, ta);

    traj->boundary[3] = s;
}

MotionState_t Trapezoid_Evaluate(const TrapezoidTrajectory_t *traj, float t) {
    if (t <= 0.0f) return traj->boundary[0];

    if (t >= traj->total_time) return traj->boundary[3];

    if (t < traj->ta) {
        return Motion_IntegrateAccel(traj->boundary[0], traj->amax, t);
    }

    if (t < traj->tc) {
        return Motion_IntegrateVelocity(traj->boundary[1], traj->boundary[1].vel,
                                        t - traj->ta);
    }

    return Motion_IntegrateAccel(traj->boundary[2], -traj->amax, t - traj->tc);
}
