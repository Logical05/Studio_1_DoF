/*
 * app.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "app/app.h"

#include "app/mode_manager.h"
#include "app/safety.h"
#include "comm/joy.h"
#include "control/joy_cmd_bridge.h"
#include "comm/charmander.h"
#include "comm/charmander_bridge.h"
#include "config/pinmap.h"
#include "control/control.h"
#include "drivers/gripper.h"
#include "drivers/motor.h"
#include "drivers/qei.h"
#include "gpio.h"
#include "usart.h"
<<<<<<< HEAD
#include "comm/aican.h"
=======
#include "modes/mode_sethome.h"
>>>>>>> 6c3e9961aa3bd8d11a4846c4cdca1c3962116e18

void App_Init(void) {
	Safety_Init();

	Motor_Init();
	QEI_Init();
	Gripper_Init();
	Control_Init();

	CharmanderBridge_Init();
	ModeManager_Init();

	JOY_Init(&huart1);
	JoyCmdBridge_Init();
}

void App_Update(void) {
	/*
	 * Synchronize safety state
	 */
	Safety_Update();

	/*
<<<<<<< HEAD
	 * Joystick input
	 */
	JOY_Update();
	JoyCmdBridge_Update();

	AICAN_Update();

	/*
=======
>>>>>>> 6c3e9961aa3bd8d11a4846c4cdca1c3962116e18
	 * Push feedback to Modbus
	 */
	CharmanderBridge_Update();
	CharmanderBridge_UpdateFeedback();

	/*
	 * Joystick input
	 */
	JOY_Update();
	JoyCmdBridge_Update();

	/*
	 * Update gripper FSM
	 */
	Gripper_Update();

	/*
	 * Set home mode has special handling since it needs to run even in emergency
	 */
	if (charmander.mode == CHARMANDER_MODE_SET_HOME) {
		Mode_SetHome_Update();
	}

	/*
	 * Emergency handling
	 */
	if (Safety_IsEmergency()) {
		Motor_Brake();
		Mode_SetHome_Update();
		return;
	}

	/*
	 * Main mode execution
	 */
	ModeManager_Update();

	/*
	 * Heartbeat LED
	 */
	HAL_GPIO_TogglePin(PIN_STATUS_LED_PORT, PIN_STATUS_LED_PIN);
}
