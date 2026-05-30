/*
 * mode_sethome.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "modes/mode_sethome.h"

#include "comm/charmander.h"
#include "control/control.h"
#include "control/trajectory_manager.h"
#include "drivers/qei.h"

void Mode_SetHome_Update(void) {
    /*
     * Reset encoder position
     */
    QEI_Reset();

    /*
     * Reset controller states
     */
    Control_Reset();

    /*
     * Clear trajectory state
     */
    Trajectory_Reset();

    /*
     * Hold zero position
     */
    Control_SetReference(0.0f, 0.0f, 0.0f);

    /*
     * Return to idle
     */
    Charmander_SetTask(0x0000);
    charmander.mode = CHARMANDER_MODE_IDLE;
}
