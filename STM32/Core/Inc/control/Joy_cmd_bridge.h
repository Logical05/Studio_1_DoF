/*
 * joy_cmd_bridge.h
 *
 *  Created on: May 30, 2026
 *      Author: rajap
 */

#ifndef INC_COMM_JOY_CMD_BRIDGE_H_
#define INC_COMM_JOY_CMD_BRIDGE_H_

/**
 * @brief  Initialise internal state (call once, after JOY_Init).
 */
void JoyCmdBridge_Init(void);

/**
 * @brief  Translate the current JoyState into charmander/safety commands.
 *         Call from the main loop AFTER JOY_Update() and BEFORE
 *         CharmanderBridge_Update() so commands are acted on in the same
 *         cycle they are issued.
 *
 *         Call order in App_Update():
 *           JOY_Update();
 *           JoyCmdBridge_Update();
 *           CharmanderBridge_Update();
 *           CharmanderBridge_UpdateFeedback();
 *           ...
 */
void JoyCmdBridge_Update(void);

#endif /* INC_COMM_JOY_CMD_BRIDGE_H_ */
