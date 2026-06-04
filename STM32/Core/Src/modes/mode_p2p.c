/*
 * mode_p2p.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "modes/mode_p2p.h"

#include "comm/charmander.h"
#include "config/robot_config.h"
#include "control/control.h"
#include "control/trajectory_manager.h"
#include "drivers/qei.h"
#include "shared/robot_math.h"

#include <stdbool.h>
#include <stdlib.h>

static float p2p_points[1];

static bool started = false;

void Mode_P2P_Update(void) {
    /*
     * Start trajectory once
     */
    if (!started) {
        int16_t target = charmander.p2p_target;

        bool cw = (target < 0);

        float rad = (charmander.p2p_unit == CHARMANDER_UNIT_INDEX)
                        ? INDEX_TO_RAD(abs(target))
                        : DEG_TO_RAD(abs(target));

        float theta = QEI_GetTheta();

        float now = wrap_2pi(theta);

        /*
         * Shortest-path style wrapping
         */
        if (cw && rad > now) {
            rad -= TWO_PI_F;
        } else if (!cw && rad < now) {
            rad += TWO_PI_F;
        }

        p2p_points[0] = theta + (rad - now);

        Trajectory_Start(p2p_points, 1, QEI_GetTheta(),TRAJ_PICK_VMAX, TRAJ_PICK_AMAX, TRAJ_PICK_JMAX, TRAJECTORY_SCURVE);

        started = true;

        Charmander_SetTask(0x0008); /* Go Point */
    }

    /*
     * Advance segment
     */
    if (Trajectory_GetState() == TRAJECTORY_WAIT) {
        Trajectory_Continue();
    }

    /*
     * Motion reference
     */
    Control_SetReference(Trajectory_GetPosition(), Trajectory_GetVelocity(),
                         Trajectory_GetAcceleration());

    /*
     * Finished
     */
    if (Trajectory_IsFinished()) {
        started = false;

        charmander.mode = CHARMANDER_MODE_IDLE;
    }
}
