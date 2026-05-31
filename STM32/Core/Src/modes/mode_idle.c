/*
 * mode_idle.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "modes/mode_idle.h"

#include "control/control.h"
#include "control/trajectory_manager.h"
#include "comm/charmander.h"

#include "drivers/qei.h"

void Mode_Idle_Enter(void)
{
	Charmander_SetTask(0x0000); /* IDLE */

	/*
	 * Stop and clear trajectory
	 */
	Trajectory_Reset();
}

void Mode_Idle_Update(void)
{
	Mode_Idle_Enter();
	/*
	 * Keep zero velocity command
	 */
	Control_SetReference(Control_GetReferencePosition(), 0.0f, 0.0f);
}
