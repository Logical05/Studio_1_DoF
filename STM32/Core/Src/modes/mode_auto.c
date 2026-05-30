/*
 * mode_auto.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */
#include "modes/mode_auto.h"

#include "comm/charmander.h"
#include "config/robot_config.h"
#include "control/control.h"
#include "control/trajectory_manager.h"
#include "drivers/gripper.h"
#include "drivers/qei.h"
#include "shared/robot_math.h"

#include <stdbool.h>
#include <stdlib.h>

#define AUTO_MAX_POINTS 16

static float auto_points[AUTO_MAX_POINTS];

static bool auto_started   = false;
static bool waiting_action = false;

static uint16_t current = 0;

static void build_auto_points(uint16_t count) {
    float theta = QEI_GetTheta();

    for (uint16_t i = 0; i < count; i++) {
        int16_t slot = charmander.pp_slots[i];

        bool cw = (slot < 0);

        float target = INDEX_TO_RAD(abs(slot));

        float now = wrap_2pi(theta);

        if (cw && (target > now)) {
            target -= TWO_PI_F;
        } else if (target < now) {
            target += TWO_PI_F;
        }

        auto_points[i] = theta + (target - now);
    }
}

void Mode_Auto_Update(void) {
    uint16_t count = charmander.pp_pair_count * 2;

    /*
     * Invalid count
     */

    if (count == 0) {
        auto_started   = false;
        waiting_action = false;

        charmander.mode = CHARMANDER_MODE_IDLE;

        return;
    }

    if (count > AUTO_MAX_POINTS) {
        count = AUTO_MAX_POINTS;
    }

    /*
     * Start trajectory once
     */

    if (!auto_started) {
        build_auto_points(count);

        Trajectory_Start(auto_points, count, QEI_GetTheta(), TRAJ_VMAX, TRAJ_AMAX);

        auto_started   = true;
        waiting_action = false;

        Charmander_SetTask(0x0002);
    }

    /*
     * Motion reference
     */

    Control_SetReference(Trajectory_GetPosition(), Trajectory_GetVelocity(),
                         Trajectory_GetAcceleration());

    /*
     * Waypoint reached
     */

    if (Trajectory_GetState() == TRAJECTORY_WAIT) {

        if (!waiting_action) {

            waiting_action = true;

            /*
             * Even index = PICK
             * Odd index  = PLACE
             */

            if (charmander.gripper_auto == CHARMANDER_ENABLE) {

                if ((current & 1U) == 0U) {

                    Gripper_Command(GRIPPER_CMD_PICK);

                    Charmander_SetTask(0x0002);
                } else {

                    Gripper_Command(GRIPPER_CMD_PLACE);

                    Charmander_SetTask(0x0004);
                }
            }
        }

        /*
         * Continue after gripper done
         */

        if (!Gripper_IsBusy()) {

            waiting_action = false;

            current++;

            Trajectory_Continue();
        }
    }

    /*
     * Sequence completed
     */

    if (Trajectory_IsFinished()) {

        auto_started   = false;
        waiting_action = false;

        current = 0;

        charmander.mode = CHARMANDER_MODE_IDLE;
    }
}
