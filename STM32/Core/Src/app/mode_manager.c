/*
 * mode_manager.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "app/mode_manager.h"

#include "comm/charmander.h"

#include "modes/mode_auto.h"
#include "modes/mode_home.h"
#include "modes/mode_idle.h"
#include "modes/mode_jog.h"
#include "modes/mode_p2p.h"
#include "modes/mode_sethome.h"
#include "modes/mode_test.h"

void ModeManager_Init(void) {
	Mode_Idle_Enter();
}

void ModeManager_Update(void) {
	switch (charmander.mode) {
		case CHARMANDER_MODE_IDLE:
			Mode_Idle_Update();
			break;

		case CHARMANDER_MODE_HOME:
			Mode_Home_Update();
			break;

		case CHARMANDER_MODE_JOG:
			Mode_Jog_Update();
			break;

		case CHARMANDER_MODE_AUTO:
			Mode_Auto_Update();
			break;

		case CHARMANDER_MODE_P2P:
			Mode_P2P_Update();
			break;

		case CHARMANDER_MODE_SET_HOME:
			Mode_SetHome_Update();
			break;

		case CHARMANDER_MODE_TEST:
			Mode_Test_Update();
			break;

		default:
			charmander.mode = CHARMANDER_MODE_IDLE;
			break;
	}
}

void ModeManager_ForceIdle(void) {
	charmander.mode = CHARMANDER_MODE_IDLE;

	Mode_Idle_Update();
}
