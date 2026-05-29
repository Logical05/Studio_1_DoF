/*
 * gripper.h
 *
 *  Created on: May 28, 2026
 *      Author: rajap
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_

#include <stdbool.h>
#include "main.h"
#include "useful.h"

/* ============================================================================
 * Config
 * ========================================================================== */

#define REED_ACTIVE_STATE     GPIO_PIN_RESET

#define GRIPPER_ON            GPIO_PIN_SET
#define GRIPPER_OFF           GPIO_PIN_RESET

#define GRIPPER_TIMEOUT_MS    2000

/* ============================================================================
 * Types
 * ========================================================================== */

typedef struct {
	Pinout_t open_out;
	Pinout_t close_out;

	Pinout_t up_out;
	Pinout_t down_out;

	Pinout_t reed_close;
	Pinout_t reed_up;
	Pinout_t reed_down;
} GripperPin_t;

typedef enum {
	GRIPPER_SEQ_IDLE = 0, GRIPPER_SEQ_PICK, GRIPPER_SEQ_PLACE
} GripperSequence_t;

typedef enum {
	GRIPPER_IDLE = 0,

	GRIPPER_OPENING, GRIPPER_CLOSING, GRIPPER_MOVING_UP, GRIPPER_MOVING_DOWN,

	/* PICK */
	GRIPPER_PICK_DOWN, GRIPPER_PICK_CLOSE, GRIPPER_PICK_UP,

	/* PLACE */
	GRIPPER_PLACE_DOWN, GRIPPER_PLACE_OPEN, GRIPPER_PLACE_UP,

	GRIPPER_SUCCESS, GRIPPER_TIMEOUT
} GripperState_t;

typedef enum {
	GRIPPER_CMD_NONE = 0,

	GRIPPER_CMD_OPEN, GRIPPER_CMD_CLOSE,

	GRIPPER_CMD_UP, GRIPPER_CMD_DOWN,

	/* high-level */

	GRIPPER_CMD_PICK, GRIPPER_CMD_PLACE

} GripperCommand_t;

typedef struct {
	GripperState_t state;

	GripperCommand_t cmd;

	uint32_t tick;

	bool busy;
} Gripper_t;

/* ============================================================================
 * Pin Mapping
 * ========================================================================== */

extern const GripperPin_t gripper_pin;

extern Gripper_t gripper;

/* ============================================================================
 * API
 * ========================================================================== */

void Gripper_Init(void);

void Gripper_Update(void);

bool Gripper_Command(GripperCommand_t cmd);

bool Gripper_IsBusy(void);

bool Gripper_IsClosed(void);
bool Gripper_IsUp(void);
bool Gripper_IsDown(void);

#endif /* INC_GRIPPER_H_ */
