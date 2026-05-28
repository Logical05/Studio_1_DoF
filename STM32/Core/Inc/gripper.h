/*
 * gripper.h
 *
 *  Created on: May 28, 2026
 *      Author: rajap
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_

#include <stdbool.h>
#include "useful.h"

/* ============================================================================
 * Types
 * ========================================================================== */

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} GripperPin_t;

typedef struct {
	GripperPin_t open_out;
	GripperPin_t close_out;
	GripperPin_t up_out;
	GripperPin_t down_out;

	GripperPin_t reed_open;
	GripperPin_t reed_close;
	GripperPin_t reed_up;
	GripperPin_t reed_down;
} GripperConfig_t;

typedef enum {
	GRIPPER_IDLE = 0,
	GRIPPER_OPENING,
	GRIPPER_CLOSING,
	GRIPPER_MOVING_UP,
	GRIPPER_MOVING_DOWN,
	GRIPPER_PICKING,
	GRIPPER_PLACING,
	GRIPPER_DONE,
	GRIPPER_ERROR
} GripperState_t;

typedef enum {
	GRIPPER_CMD_NONE = 0,
	GRIPPER_CMD_OPEN,
	GRIPPER_CMD_CLOSE,
	GRIPPER_CMD_UP,
	GRIPPER_CMD_DOWN,
	GRIPPER_CMD_PICK,
	GRIPPER_CMD_PLACE
} GripperCommand_t;

/* ============================================================================
 * API
 * ========================================================================== */

void Gripper_Init(const GripperConfig_t *config);

void Gripper_Update(void);

void Gripper_Command(GripperCommand_t cmd);

void Gripper_Stop(void);

bool Gripper_IsBusy(void);

bool Gripper_IsDone(void);

bool Gripper_HasError(void);

GripperState_t Gripper_GetState(void);

/* ============================================================================
 * Sensors
 * ========================================================================== */

bool Gripper_IsOpened(void);
bool Gripper_IsClosed(void);

bool Gripper_IsUp(void);
bool Gripper_IsDown(void);

#endif /* INC_GRIPPER_H_ */
