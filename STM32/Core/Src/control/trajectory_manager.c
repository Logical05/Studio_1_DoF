/*
 * trajectory_manager.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "control/trajectory_manager.h"

#include "control/trajectory/trajectory_scurve.h"
#include "control/trajectory/trajectory_trapezoid.h"
#include "math.h"
#include "shared/robot_math.h"
#include "stm32g4xx_hal.h"

Trajectory_t trajectory = {0};

static SCurveTrajectory_t    scurve;
static TrapezoidTrajectory_t trapezoid;

/* ============================================================
 * Internal
 * ============================================================ */

static void load_segment(void) {
    trajectory.qf = trajectory.points[trajectory.current];

    trajectory.distance = fabsf(trajectory.qf - trajectory.q0);

    trajectory.direction = signf_fast(trajectory.qf - trajectory.q0);

    trajectory.t = 0.0f;

    if (trajectory.mode == TRAJECTORY_SCURVE) {
        SCurve_Build(&scurve, trajectory.distance, trajectory.vmax, trajectory.amax,
                     trajectory.jmax);
    } else {
        Trapezoid_Build(&trapezoid, trajectory.distance, trajectory.vmax,
                        trajectory.amax);
    }
}

static float profile_total_time(void) {
    if (trajectory.mode == TRAJECTORY_SCURVE) {
        return scurve.total_time;
    }

    return trapezoid.total_time;
}

static MotionState_t profile_eval(float t) {
    if (trajectory.mode == TRAJECTORY_SCURVE) {
        return SCurve_Evaluate(&scurve, t);
    }

    return Trapezoid_Evaluate(&trapezoid, t);
}

/* ============================================================
 * Public API
 * ============================================================ */

void Trajectory_Reset(void) {
    __disable_irq();

    trajectory.points = 0;

    trajectory.num_points = 0;
    trajectory.current    = 0;

    trajectory.state = TRAJECTORY_DONE;
    trajectory.mode  = TRAJECTORY_SCURVE;

    trajectory.q0 = 0.0f;
    trajectory.qf = 0.0f;

    trajectory.vmax = 0.0f;
    trajectory.amax = 0.0f;
    trajectory.jmax = 0.0f;

    trajectory.distance  = 0.0f;
    trajectory.direction = 1.0f;

    trajectory.t = 0.0f;

    __enable_irq();
}

void Trajectory_Start(const float *points, uint16_t count, float current_position,
                      float vmax, float amax, float jmax, TrajectoryMode_t mode) {
    if (points == 0 || count == 0) {
        Trajectory_Reset();
        return;
    }

    __disable_irq();

    trajectory.points = points;

    trajectory.num_points = count;
    trajectory.current    = 0;

    trajectory.mode = mode;

    trajectory.q0 = current_position;

    trajectory.vmax = vmax;
    trajectory.amax = amax;
    trajectory.jmax = jmax;

    load_segment();

    trajectory.state = TRAJECTORY_RUN;
    __enable_irq();
}

void Trajectory_Update(float dt) {
    if (trajectory.state != TRAJECTORY_RUN) return;

    trajectory.t += dt;

    float total = profile_total_time();

    if (trajectory.t >= total) {
        trajectory.t     = total;
        trajectory.state = TRAJECTORY_WAIT;
    }
}

void Trajectory_Continue(void) {
    __disable_irq();

    if (trajectory.state != TRAJECTORY_WAIT) {
        __enable_irq();
        return;
    }

    trajectory.current++;

    if (trajectory.current >= trajectory.num_points) {
        trajectory.current = trajectory.num_points - 1;

        trajectory.state = TRAJECTORY_DONE;
        __enable_irq();
        return;
    }

    trajectory.q0 = trajectory.qf;
    load_segment();

    trajectory.state = TRAJECTORY_RUN;

    __enable_irq();
}

void Trajectory_SetLimits(float vmax, float amax, float jmax) {
    trajectory.vmax = vmax;
    trajectory.amax = amax;
    trajectory.jmax = jmax;
}

bool Trajectory_IsFinished(void) { return trajectory.state == TRAJECTORY_DONE; }

TrajectoryState_t Trajectory_GetState(void) { return trajectory.state; }

float Trajectory_GetPosition(void) {
    MotionState_t s = profile_eval(trajectory.t);
    return trajectory.q0 + trajectory.direction * s.pos;
}

float Trajectory_GetVelocity(void) {
    MotionState_t s = profile_eval(trajectory.t);
    return trajectory.direction * s.vel;
}

float Trajectory_GetAcceleration(void) {
    MotionState_t s = profile_eval(trajectory.t);
    return trajectory.direction * s.acc;
}
