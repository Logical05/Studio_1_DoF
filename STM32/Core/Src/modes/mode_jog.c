/*
 * mode_jog.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "modes/mode_jog.h"

#include "comm/charmander.h"
#include "config/robot_config.h"
#include "control/control.h"
#include "control/trajectory_manager.h"
#include "drivers/qei.h"
#include "shared/robot_math.h"

static float jog_points[1];

static bool started = false;

void Mode_Jog_Update(void) {
    /*
     * Start trajectory once
     */
    if (!started) {
        if (charmander.jog_degrees == 0) {
            charmander.mode = CHARMANDER_MODE_IDLE;
            return;
        }

        Charmander_SetTask(0x0008); /* Go Point */
        float target = DEG_TO_RAD((float)charmander.jog_degrees);

        jog_points[0] = Control_GetReferencePosition() + target;

        Trajectory_Start(jog_points, 1, QEI_GetTheta(),TRAJ_PICK_VMAX, TRAJ_PICK_AMAX, TRAJ_PICK_JMAX);

        started = true;
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

        /*
         * Clear command
         */
        charmander.jog_degrees = 0;

        charmander.mode = CHARMANDER_MODE_IDLE;
    }
}
