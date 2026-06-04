/*
 * traj_scurve.c
 *
 *  Created on: Jun 3, 2026
 *      Author: rajap
 */

#include "control/trajectory/trajectory_scurve.h"

#include "math.h"

static void SCurve_BuildJerkOnly(SCurveTrajectory_t *traj, float distance,
                                 float jmax) {
    float Tj = cbrtf(distance / (2.0f * jmax));

    traj->t1 = Tj;
    traj->t2 = 2.0f * Tj;
    traj->t3 = 3.0f * Tj;
    traj->t4 = 4.0f * Tj;

    traj->t5 = traj->t4;
    traj->t6 = traj->t4;
    traj->t7 = traj->t4;

    traj->total_time = traj->t4;

    MotionState_t s = {0};

    traj->boundary[0] = s;

    s                 = Motion_IntegrateJerk(s, +jmax, Tj);
    traj->boundary[1] = s;

    s                 = Motion_IntegrateJerk(s, -jmax, Tj);
    traj->boundary[2] = s;

    s                 = Motion_IntegrateJerk(s, -jmax, Tj);
    traj->boundary[3] = s;

    s                 = Motion_IntegrateJerk(s, +jmax, Tj);
    traj->boundary[4] = s;

    traj->boundary[5] = s;
    traj->boundary[6] = s;
    traj->boundary[7] = s;
}

static void SCurve_BuildNoCruise(SCurveTrajectory_t *traj, float distance,
                                 float amax, float jmax) {
    float Tj = amax / jmax;

    float Ta = (-3.0f * Tj + sqrtf(Tj * Tj + 4.0f * distance / amax)) * 0.5f;

    if (Ta < 0.0f) {
        Ta = 0.0f;
    }

    traj->t1 = Tj;
    traj->t2 = traj->t1 + Ta;
    traj->t3 = traj->t2 + Tj;

    traj->t4 = traj->t3;

    traj->t5 = traj->t4 + Tj;
    traj->t6 = traj->t5 + Ta;
    traj->t7 = traj->t6 + Tj;

    traj->total_time = traj->t7;

    MotionState_t s = {0};

    traj->boundary[0] = s;

    s                 = Motion_IntegrateJerk(s, +jmax, Tj);
    traj->boundary[1] = s;

    s                 = Motion_IntegrateAccel(s, +amax, Ta);
    traj->boundary[2] = s;

    s                 = Motion_IntegrateJerk(s, -jmax, Tj);
    traj->boundary[3] = s;

    traj->boundary[4] = s;

    s                 = Motion_IntegrateJerk(s, -jmax, Tj);
    traj->boundary[5] = s;

    s                 = Motion_IntegrateAccel(s, -amax, Ta);
    traj->boundary[6] = s;

    s                 = Motion_IntegrateJerk(s, +jmax, Tj);
    traj->boundary[7] = s;
}

static void SCurve_BuildFullProfile(SCurveTrajectory_t *traj, float distance,
                                    float vmax, float amax, float jmax) {
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

    traj->t1 = Tj;
    traj->t2 = traj->t1 + Ta;
    traj->t3 = traj->t2 + Tj;
    traj->t4 = traj->t3 + Tc;
    traj->t5 = traj->t4 + Tj;
    traj->t6 = traj->t5 + Ta;
    traj->t7 = traj->t6 + Tj;

    traj->total_time = traj->t7;

    MotionState_t s = {0};

    traj->boundary[0] = s;

    s                 = Motion_IntegrateJerk(s, +jmax, Tj);
    traj->boundary[1] = s;

    s                 = Motion_IntegrateAccel(s, +amax, Ta);
    traj->boundary[2] = s;

    s                 = Motion_IntegrateJerk(s, -jmax, Tj);
    traj->boundary[3] = s;

    s                 = Motion_IntegrateVelocity(s, s.vel, Tc);
    traj->boundary[4] = s;

    s                 = Motion_IntegrateJerk(s, -jmax, Tj);
    traj->boundary[5] = s;

    s                 = Motion_IntegrateAccel(s, -amax, Ta);
    traj->boundary[6] = s;

    s                 = Motion_IntegrateJerk(s, +jmax, Tj);
    traj->boundary[7] = s;
}

MotionState_t SCurve_Evaluate(const SCurveTrajectory_t *traj, float t) {
    if (t <= 0.0f) {
        return traj->boundary[0];
    }

    if (t >= traj->total_time) {
        return traj->boundary[7];
    }

    switch (traj->profile) {
        /* =====================================================
         * JERK ONLY
         * +J -J -J +J
         * ===================================================== */
        case PROFILE_JERK_ONLY:
            {
                if (t < traj->t1) {
                    return Motion_IntegrateJerk(traj->boundary[0], +traj->jmax, t);
                }

                if (t < traj->t2) {
                    return Motion_IntegrateJerk(traj->boundary[1], -traj->jmax,
                                                t - traj->t1);
                }

                if (t < traj->t3) {
                    return Motion_IntegrateJerk(traj->boundary[2], -traj->jmax,
                                                t - traj->t2);
                }

                return Motion_IntegrateJerk(traj->boundary[3], +traj->jmax,
                                            t - traj->t3);
            }

            /* =====================================================
             * NO CRUISE
             * +J +A -J -J -A +J
             * ===================================================== */
        case PROFILE_NO_CRUISE:
            {
                if (t < traj->t1) {
                    return Motion_IntegrateJerk(traj->boundary[0], +traj->jmax, t);
                }

                if (t < traj->t2) {
                    return Motion_IntegrateAccel(traj->boundary[1], +traj->amax,
                                                 t - traj->t1);
                }

                if (t < traj->t3) {
                    return Motion_IntegrateJerk(traj->boundary[2], -traj->jmax,
                                                t - traj->t2);
                }

                if (t < traj->t5) {
                    return Motion_IntegrateJerk(traj->boundary[4], -traj->jmax,
                                                t - traj->t4);
                }

                if (t < traj->t6) {
                    return Motion_IntegrateAccel(traj->boundary[5], -traj->amax,
                                                 t - traj->t5);
                }

                return Motion_IntegrateJerk(traj->boundary[6], +traj->jmax,
                                            t - traj->t6);
            }

            /* =====================================================
             * FULL PROFILE
             * +J +A -J V -J -A +J
             * ===================================================== */
        default:
            {
                if (t < traj->t1) {
                    return Motion_IntegrateJerk(traj->boundary[0], +traj->jmax, t);
                }

                if (t < traj->t2) {
                    return Motion_IntegrateAccel(traj->boundary[1], +traj->amax,
                                                 t - traj->t1);
                }

                if (t < traj->t3) {
                    return Motion_IntegrateJerk(traj->boundary[2], -traj->jmax,
                                                t - traj->t2);
                }

                if (t < traj->t4) {
                    return Motion_IntegrateVelocity(
                        traj->boundary[3], traj->boundary[3].vel, t - traj->t3);
                }

                if (t < traj->t5) {
                    return Motion_IntegrateJerk(traj->boundary[4], -traj->jmax,
                                                t - traj->t4);
                }

                if (t < traj->t6) {
                    return Motion_IntegrateAccel(traj->boundary[5], -traj->amax,
                                                 t - traj->t5);
                }

                return Motion_IntegrateJerk(traj->boundary[6], +traj->jmax,
                                            t - traj->t6);
            }
    }
    return traj->boundary[7];
}

static SCurveProfile_t SCurve_SelectProfile(float distance, float vmax, float amax,
                                            float jmax) {
    float Tj = amax / jmax;

    float Ta = vmax / amax - Tj;

    if (Ta < 0.0f) {
        Ta = 0.0f;
    }

    float da = (amax * amax * amax) / (jmax * jmax) +
               1.5f * (amax * amax / jmax) * Ta + 0.5f * amax * Ta * Ta;

    float d_ad = 2.0f * da;

    float apeak = sqrtf(vmax * jmax);

    if (amax > apeak) {
        return PROFILE_NO_CRUISE;
    }

    if (distance < d_ad) {
        return PROFILE_NO_CRUISE;
    }

    return PROFILE_FULL;
}

void SCurve_Build(SCurveTrajectory_t *traj, float distance, float vmax, float amax,
                  float jmax) {
    traj->distance = distance;
    traj->vmax     = vmax;
    traj->amax     = amax;
    traj->jmax     = jmax;

    traj->profile = SCurve_SelectProfile(distance, vmax, amax, jmax);

    switch (traj->profile) {
        case PROFILE_JERK_ONLY: SCurve_BuildJerkOnly(traj, distance, jmax); break;

        case PROFILE_NO_CRUISE:
            SCurve_BuildNoCruise(traj, distance, amax, jmax);
            break;

        default: SCurve_BuildFullProfile(traj, distance, vmax, amax, jmax); break;
    }
}
