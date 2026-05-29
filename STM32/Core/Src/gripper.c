#include "gripper.h"

/* ============================================================================
 * Pin Map
 * ========================================================================== */

const GripperPin_t gripper_pin = { { GPIOC, GPIO_PIN_13 },   // OPEN
	{ GPIOB, GPIO_PIN_14 },   // CLOSE

	{ GPIOB, GPIO_PIN_7 },   // UP
	{ GPIOA, GPIO_PIN_15 },   // DOWN

	{ GPIOB, GPIO_PIN_15 },   // REED CLOSE
	{ GPIOB, GPIO_PIN_2 },   // REED UP
	{ GPIOB, GPIO_PIN_1 }    // REED DOWN
};

/* ============================================================================
 * Global
 * ========================================================================== */

Gripper_t gripper;

/* ============================================================================
 * Internal
 * ========================================================================== */

static void Gripper_StopOutputs(void) {
	HAL_GPIO_WritePin(gripper_pin.open_out.port, gripper_pin.open_out.pin,
	GRIPPER_OFF);

	HAL_GPIO_WritePin(gripper_pin.close_out.port, gripper_pin.close_out.pin,
	GRIPPER_OFF);

	HAL_GPIO_WritePin(gripper_pin.up_out.port, gripper_pin.up_out.pin,
	GRIPPER_OFF);

	HAL_GPIO_WritePin(gripper_pin.down_out.port, gripper_pin.down_out.pin,
	GRIPPER_OFF);
}

/* ============================================================================
 * Reed Status
 * ========================================================================== */
bool Gripper_IsOpened(void) {
	return !Gripper_IsClosed();
}

bool Gripper_IsClosed(void) {
	return HAL_GPIO_ReadPin(gripper_pin.reed_close.port, gripper_pin.reed_close.pin) == REED_ACTIVE_STATE;
}

bool Gripper_IsUp(void) {
	return HAL_GPIO_ReadPin(gripper_pin.reed_up.port, gripper_pin.reed_up.pin) == REED_ACTIVE_STATE;
}

bool Gripper_IsDown(void) {
	return HAL_GPIO_ReadPin(gripper_pin.reed_down.port, gripper_pin.reed_down.pin) == REED_ACTIVE_STATE;
}

/* ============================================================================
 * Init
 * ========================================================================== */

void Gripper_Init(void) {
	Gripper_StopOutputs();

	gripper.state = GRIPPER_IDLE;
	gripper.busy = false;
}

/* ============================================================================
 * Command
 * ========================================================================== */
bool Gripper_Command(GripperCommand_t cmd) {
	if (gripper.busy) {
		return false;
	}

	Gripper_StopOutputs();

	gripper.busy = true;

	gripper.cmd = cmd;
	gripper.tick = HAL_GetTick();

	switch (cmd) {

		/* =========================================================
		 * LOW LEVEL
		 * =======================================================*/

		case GRIPPER_CMD_OPEN:
			HAL_GPIO_WritePin(gripper_pin.open_out.port, gripper_pin.open_out.pin,
			GRIPPER_ON);

			gripper.state = GRIPPER_OPENING;
			break;
		case GRIPPER_CMD_CLOSE:
			HAL_GPIO_WritePin(gripper_pin.close_out.port, gripper_pin.close_out.pin,
			GRIPPER_ON);

			gripper.state = GRIPPER_CLOSING;
			break;
		case GRIPPER_CMD_UP:
			HAL_GPIO_WritePin(gripper_pin.up_out.port, gripper_pin.up_out.pin,
			GRIPPER_ON);

			gripper.state = GRIPPER_MOVING_UP;
			break;
		case GRIPPER_CMD_DOWN:
			HAL_GPIO_WritePin(gripper_pin.down_out.port, gripper_pin.down_out.pin,
			GRIPPER_ON);

			gripper.state = GRIPPER_MOVING_DOWN;
			break;

			/* =========================================================
			 * HIGH LEVEL
			 * =======================================================*/

		case GRIPPER_CMD_PICK:
			HAL_GPIO_WritePin(gripper_pin.down_out.port, gripper_pin.down_out.pin,
			GRIPPER_ON);

			gripper.state = GRIPPER_PICK_DOWN;
			break;

		case GRIPPER_CMD_PLACE:
			HAL_GPIO_WritePin(gripper_pin.down_out.port, gripper_pin.down_out.pin,
			GRIPPER_ON);

			gripper.state = GRIPPER_PLACE_DOWN;
			break;
		default:
			gripper.busy = false;
			return false;
	}
	return true;
}
/* ============================================================================
 * Update
 * ========================================================================== */
void Gripper_Update(void) {
	if (!gripper.busy) {
		return;
	}

	/* =========================================================
	 * TIMEOUT
	 * =======================================================*/

	if ((HAL_GetTick() - gripper.tick) >= GRIPPER_TIMEOUT_MS) {
		Gripper_StopOutputs();
		gripper.state = GRIPPER_TIMEOUT;
		gripper.busy = false;
		return;
	}

	switch (gripper.state) {

		/* =====================================================
		 * LOW LEVEL
		 * ===================================================*/

		case GRIPPER_OPENING:
			if (Gripper_IsOpened()) {
				Gripper_StopOutputs();

				gripper.state = GRIPPER_SUCCESS;
				gripper.busy = false;
			}
			break;
		case GRIPPER_CLOSING:
			if (Gripper_IsClosed()) {
				Gripper_StopOutputs();

				gripper.state = GRIPPER_SUCCESS;
				gripper.busy = false;
			}
			break;
		case GRIPPER_MOVING_UP:
			if (Gripper_IsUp()) {
				Gripper_StopOutputs();

				gripper.state = GRIPPER_SUCCESS;
				gripper.busy = false;
			}
			break;
		case GRIPPER_MOVING_DOWN:
			if (Gripper_IsDown()) {
				Gripper_StopOutputs();

				gripper.state = GRIPPER_SUCCESS;
				gripper.busy = false;
			}
			break;

			/* =====================================================
			 * PICK SEQUENCE
			 * ===================================================*/

		case GRIPPER_PICK_DOWN:
			if (Gripper_IsDown()) {
				Gripper_StopOutputs();

				HAL_GPIO_WritePin(gripper_pin.close_out.port,
					gripper_pin.close_out.pin,
					GRIPPER_ON);

				gripper.state = GRIPPER_PICK_CLOSE;
			}
			break;
		case GRIPPER_PICK_CLOSE:
			if (Gripper_IsClosed()) {
				Gripper_StopOutputs();

				HAL_GPIO_WritePin(gripper_pin.up_out.port, gripper_pin.up_out.pin,
				GRIPPER_ON);

				gripper.state = GRIPPER_PICK_UP;
			}
			break;
		case GRIPPER_PICK_UP:
			if (Gripper_IsUp()) {
				Gripper_StopOutputs();

				gripper.state = GRIPPER_SUCCESS;
				gripper.busy = false;
			}
			break;

			/* =====================================================
			 * PLACE SEQUENCE
			 * ===================================================*/

		case GRIPPER_PLACE_DOWN:
			if (Gripper_IsDown()) {
				Gripper_StopOutputs();

				HAL_GPIO_WritePin(gripper_pin.open_out.port,
					gripper_pin.open_out.pin,
					GRIPPER_ON);

				gripper.state = GRIPPER_PLACE_OPEN;
			}
			break;
		case GRIPPER_PLACE_OPEN:
			if (Gripper_IsOpened()) {
				Gripper_StopOutputs();

				HAL_GPIO_WritePin(gripper_pin.up_out.port, gripper_pin.up_out.pin,
				GRIPPER_ON);

				gripper.state = GRIPPER_PLACE_UP;
			}
			break;
		case GRIPPER_PLACE_UP:
			if (Gripper_IsUp()) {
				Gripper_StopOutputs();

				gripper.state = GRIPPER_SUCCESS;
				gripper.busy = false;
			}
			break;
		default:
			break;
	}
}

/* ============================================================================
 * Busy
 * ========================================================================== */

bool Gripper_IsBusy(void) {
	return gripper.busy;
}
