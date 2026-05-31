/*
 * trajectory_manager.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "control/trajectory_manager.h"

#include "math.h"
#include "shared/robot_math.h"
#include "stm32g4xx_hal.h"

Trajectory_t trajectory = {0};

/* ============================================================
 * S-curve trajectory equations
 * ============================================================ */

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

static void SCurve_BuildJerkOnly(float distance, float jmax) {
    float Tj = cbrtf(distance / (2.0f * jmax));

    trajectory.t1 = Tj;
    trajectory.t2 = 2.0f * Tj;
    trajectory.t3 = 3.0f * Tj;
    trajectory.t4 = 4.0f * Tj;

    trajectory.t5 = trajectory.t4;
    trajectory.t6 = trajectory.t4;
    trajectory.t7 = trajectory.t4;

    trajectory.total_time = trajectory.t4;

    MotionState_t s = {0};

    trajectory.boundary[0] = s;

    s                      = Motion_IntegrateJerk(s, +jmax, Tj);
    trajectory.boundary[1] = s;

    s                      = Motion_IntegrateJerk(s, -jmax, Tj);
    trajectory.boundary[2] = s;

    s                      = Motion_IntegrateJerk(s, -jmax, Tj);
    trajectory.boundary[3] = s;

    s                      = Motion_IntegrateJerk(s, +jmax, Tj);
    trajectory.boundary[4] = s;

    trajectory.boundary[5] = s;
    trajectory.boundary[6] = s;
    trajectory.boundary[7] = s;
}

static void SCurve_BuildNoCruise(float distance, float amax, float jmax) {
    float Tj = amax / jmax;

    float Ta = (-3.0f * Tj + sqrtf(Tj * Tj + 4.0f * distance / amax)) * 0.5f;

    if (Ta < 0.0f) {
        Ta = 0.0f;
    }

    trajectory.t1 = Tj;
    trajectory.t2 = trajectory.t1 + Ta;
    trajectory.t3 = trajectory.t2 + Tj;

    trajectory.t4 = trajectory.t3;

    trajectory.t5 = trajectory.t4 + Tj;
    trajectory.t6 = trajectory.t5 + Ta;
    trajectory.t7 = trajectory.t6 + Tj;

    trajectory.total_time = trajectory.t7;

    MotionState_t s = {0};

    trajectory.boundary[0] = s;

    s                      = Motion_IntegrateJerk(s, +jmax, Tj);
    trajectory.boundary[1] = s;

    s                      = Motion_IntegrateAccel(s, +amax, Ta);
    trajectory.boundary[2] = s;

    s                      = Motion_IntegrateJerk(s, -jmax, Tj);
    trajectory.boundary[3] = s;

    trajectory.boundary[4] = s;

    s                      = Motion_IntegrateJerk(s, -jmax, Tj);
    trajectory.boundary[5] = s;

    s                      = Motion_IntegrateAccel(s, -amax, Ta);
    trajectory.boundary[6] = s;

    s                      = Motion_IntegrateJerk(s, +jmax, Tj);
    trajectory.boundary[7] = s;
}

static void SCurve_BuildFullProfile(float distance, float vmax, float amax,
                                    float jmax) {
    float Tj = amax / jmax;

    float Ta = vmax / amax - Tj;
    if (Ta < 0.0f) {
        Ta = 0.0f;
    }

    float da = ((amax * amax * amax) / (jmax * jmax)) +
               (1.5f * (amax * amax / jmax)) * Ta + (0.5f * amax * Ta * Ta);

    float d_ad = 2.0f * da;

    float Tc = 0.0f;

    if (distance > d_ad) {
        Tc = (distance - d_ad) / vmax;
    }

    trajectory.t1 = Tj;
    trajectory.t2 = trajectory.t1 + Ta;
    trajectory.t3 = trajectory.t2 + Tj;
    trajectory.t4 = trajectory.t3 + Tc;
    trajectory.t5 = trajectory.t4 + Tj;
    trajectory.t6 = trajectory.t5 + Ta;
    trajectory.t7 = trajectory.t6 + Tj;

    trajectory.total_time = trajectory.t7;

    MotionState_t s = {0};

    trajectory.boundary[0] = s;

    s                      = Motion_IntegrateJerk(s, +jmax, Tj);
    trajectory.boundary[1] = s;

    s                      = Motion_IntegrateAccel(s, +amax, Ta);
    trajectory.boundary[2] = s;

    s                      = Motion_IntegrateJerk(s, -jmax, Tj);
    trajectory.boundary[3] = s;

    s                      = Motion_IntegrateVelocity(s, s.vel, Tc);
    trajectory.boundary[4] = s;

    s                      = Motion_IntegrateJerk(s, -jmax, Tj);
    trajectory.boundary[5] = s;

    s                      = Motion_IntegrateAccel(s, -amax, Ta);
    trajectory.boundary[6] = s;

    s                      = Motion_IntegrateJerk(s, +jmax, Tj);
    trajectory.boundary[7] = s;
}

static MotionState_t SCurve_Evaluate(float t) {
    if (t <= 0.0f) {
        return trajectory.boundary[0];
    }

    if (t >= trajectory.total_time) {
        return trajectory.boundary[7];
    }

    switch (trajectory.profile) {
        /* =====================================================
         * JERK ONLY
         * +J -J -J +J
         * ===================================================== */
        case PROFILE_JERK_ONLY:
            {
                if (t < trajectory.t1) {
                    return Motion_IntegrateJerk(trajectory.boundary[0],
                                                +trajectory.jmax, t);
                }

                if (t < trajectory.t2) {
                    return Motion_IntegrateJerk(trajectory.boundary[1],
                                                -trajectory.jmax, t - trajectory.t1);
                }

                if (t < trajectory.t3) {
                    return Motion_IntegrateJerk(trajectory.boundary[2],
                                                -trajectory.jmax, t - trajectory.t2);
                }

                return Motion_IntegrateJerk(trajectory.boundary[3], +trajectory.jmax,
                                            t - trajectory.t3);
            }

            /* =====================================================
             * NO CRUISE
             * +J +A -J -J -A +J
             * ===================================================== */
        case PROFILE_NO_CRUISE:
            {
                if (t < trajectory.t1) {
                    return Motion_IntegrateJerk(trajectory.boundary[0],
                                                +trajectory.jmax, t);
                }

                if (t < trajectory.t2) {
                    return Motion_IntegrateAccel(
                        trajectory.boundary[1], +trajectory.amax, t - trajectory.t1);
                }

                if (t < trajectory.t3) {
                    return Motion_IntegrateJerk(trajectory.boundary[2],
                                                -trajectory.jmax, t - trajectory.t2);
                }

                if (t < trajectory.t5) {
                    return Motion_IntegrateJerk(trajectory.boundary[4],
                                                -trajectory.jmax, t - trajectory.t4);
                }

                if (t < trajectory.t6) {
                    return Motion_IntegrateAccel(
                        trajectory.boundary[5], -trajectory.amax, t - trajectory.t5);
                }

                return Motion_IntegrateJerk(trajectory.boundary[6], +trajectory.jmax,
                                            t - trajectory.t6);
            }

            /* =====================================================
             * FULL PROFILE
             * +J +A -J V -J -A +J
             * ===================================================== */
        default:
            {
                if (t < trajectory.t1) {
                    return Motion_IntegrateJerk(trajectory.boundary[0],
                                                +trajectory.jmax, t);
                }

                if (t < trajectory.t2) {
                    return Motion_IntegrateAccel(
                        trajectory.boundary[1], +trajectory.amax, t - trajectory.t1);
                }

                if (t < trajectory.t3) {
                    return Motion_IntegrateJerk(trajectory.boundary[2],
                                                -trajectory.jmax, t - trajectory.t2);
                }

                if (t < trajectory.t4) {
                    return Motion_IntegrateVelocity(trajectory.boundary[3],
                                                    trajectory.boundary[3].vel,
                                                    t - trajectory.t3);
                }

                if (t < trajectory.t5) {
                    return Motion_IntegrateJerk(trajectory.boundary[4],
                                                -trajectory.jmax, t - trajectory.t4);
                }

                if (t < trajectory.t6) {
                    return Motion_IntegrateAccel(
                        trajectory.boundary[5], -trajectory.amax, t - trajectory.t5);
                }

                return Motion_IntegrateJerk(trajectory.boundary[6], +trajectory.jmax,
                                            t - trajectory.t6);
            }
    }
    return trajectory.boundary[7];
}

static ProfileType_t SCurve_SelectProfile(float distance) {
    float Tj = trajectory.amax / trajectory.jmax;

    float Ta = trajectory.vmax / trajectory.amax - Tj;

    if (Ta < 0.0f) {
        Ta = 0.0f;
    }

    float da = (trajectory.amax * trajectory.amax * trajectory.amax) /
                   (trajectory.jmax * trajectory.jmax) +
               1.5f * (trajectory.amax * trajectory.amax / trajectory.jmax) * Ta +
               0.5f * trajectory.amax * Ta * Ta;

    float d_ad = 2.0f * da;

    float apeak = sqrtf(trajectory.vmax * trajectory.jmax);

    if (trajectory.amax > apeak) {
        return PROFILE_NO_CRUISE;
    }

    if (distance < d_ad) {
        return PROFILE_NO_CRUISE;
    }

    return PROFILE_FULL;
}

/* ============================================================
 * Internal
 * ============================================================ */

static void load_segment(void) {
    trajectory.qf = trajectory.points[trajectory.current];

    trajectory.distance = fabsf(trajectory.qf - trajectory.q0);

    trajectory.direction = signf_fast(trajectory.qf - trajectory.q0);

    trajectory.t = 0.0f;

    trajectory.profile = SCurve_SelectProfile(trajectory.distance);

    switch (trajectory.profile) {
        case PROFILE_JERK_ONLY:
            SCurve_BuildJerkOnly(trajectory.distance, trajectory.jmax);
            break;

        case PROFILE_NO_CRUISE:
            SCurve_BuildNoCruise(trajectory.distance, trajectory.amax,
                                 trajectory.jmax);
            break;

        default:
            SCurve_BuildFullProfile(trajectory.distance, trajectory.vmax,
                                    trajectory.amax, trajectory.jmax);
            break;
    }
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

    trajectory.q0 = 0.0f;
    trajectory.qf = 0.0f;

    trajectory.vmax = 0.0f;
    trajectory.amax = 0.0f;
    trajectory.jmax = 0.0f;

    trajectory.distance  = 0.0f;
    trajectory.direction = 1.0f;

    trajectory.total_time = 0.0f;
    trajectory.t          = 0.0f;

    trajectory.boundary[0].pos = 0.0f;
    trajectory.boundary[0].vel = 0.0f;
    trajectory.boundary[0].acc = 0.0f;

    trajectory.boundary[1].pos = 0.0f;
    trajectory.boundary[1].vel = 0.0f;
    trajectory.boundary[1].acc = 0.0f;

    trajectory.boundary[2].pos = 0.0f;
    trajectory.boundary[2].vel = 0.0f;
    trajectory.boundary[2].acc = 0.0f;

    trajectory.boundary[3].pos = 0.0f;
    trajectory.boundary[3].vel = 0.0f;
    trajectory.boundary[3].acc = 0.0f;

    trajectory.boundary[4].pos = 0.0f;
    trajectory.boundary[4].vel = 0.0f;
    trajectory.boundary[4].acc = 0.0f;

    trajectory.boundary[5].pos = 0.0f;
    trajectory.boundary[5].vel = 0.0f;
    trajectory.boundary[5].acc = 0.0f;

    trajectory.boundary[6].pos = 0.0f;
    trajectory.boundary[6].vel = 0.0f;
    trajectory.boundary[6].acc = 0.0f;

    trajectory.boundary[7].pos = 0.0f;
    trajectory.boundary[7].vel = 0.0f;
    trajectory.boundary[7].acc = 0.0f;

    __enable_irq();
}

void Trajectory_Start(const float *points, uint16_t count, float current_position,
                      float vmax, float amax, float jmax) {
    if (points == 0 || count == 0) {
        Trajectory_Reset();
        return;
    }

    __disable_irq();

    trajectory.points = points;

    trajectory.num_points = count;
    trajectory.current    = 0;

    trajectory.q0 = current_position;

    trajectory.vmax = vmax;
    trajectory.amax = amax;
    trajectory.jmax = jmax;

    load_segment();

    trajectory.state = TRAJECTORY_RUN;
    __enable_irq();
}

void Trajectory_Update(float dt) {
    if (trajectory.state != TRAJECTORY_RUN) {
        return;
    }

    trajectory.t += dt;

    if (trajectory.t >= trajectory.total_time) {
        trajectory.t = trajectory.total_time;

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
    MotionState_t s = SCurve_Evaluate(trajectory.t);

    float pos = trajectory.q0 + trajectory.direction * s.pos;

    float min_pos = fminf(trajectory.q0, trajectory.qf);
    float max_pos = fmaxf(trajectory.q0, trajectory.qf);

    if (pos < min_pos) pos = min_pos;
    if (pos > max_pos) pos = max_pos;

    return pos;
}

float Trajectory_GetVelocity(void) {
    MotionState_t s = SCurve_Evaluate(trajectory.t);
    return trajectory.direction * s.vel;
}

float Trajectory_GetAcceleration(void) {
    MotionState_t s = SCurve_Evaluate(trajectory.t);
    return trajectory.direction * s.acc;
}
