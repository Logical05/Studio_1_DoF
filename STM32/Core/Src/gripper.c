/*
 * gripper.c
 *
 *  Created on: May 28, 2026
 *      Author: rajap
 */

#include "gripper.h"

/* ============================================================================
 * Config
 * ========================================================================== */

#define GRIPPER_TIMEOUT_MS 2000U

/* ============================================================================
 * Private data
 * ========================================================================== */

static GripperConfig_t g_cfg;

static GripperState_t g_state = GRIPPER_IDLE;

static uint32_t g_timestamp = 0;

/* ============================================================================
 * Helpers
 * ========================================================================== */

static inline void pin_write(const GripperPin_t *p, GPIO_PinState state) {
	HAL_GPIO_WritePin(p->port, p->pin, state);
}

static inline GPIO_PinState pin_read(const GripperPin_t *p) {
	return HAL_GPIO_ReadPin(p->port, p->pin);
}

static void all_outputs_off(void) {
	pin_write(&g_cfg.open_out, GPIO_PIN_RESET);
	pin_write(&g_cfg.close_out, GPIO_PIN_RESET);
	pin_write(&g_cfg.up_out, GPIO_PIN_RESET);
	pin_write(&g_cfg.down_out, GPIO_PIN_RESET);
}

static void timeout_reset(void) {
	g_timestamp = HAL_GetTick();
}

static bool timeout_expired(void) {
	return (HAL_GetTick() - g_timestamp) > GRIPPER_TIMEOUT_MS;
}

/* ============================================================================
 * Low-level actions
 * ========================================================================== */

static void action_open(void) {
	all_outputs_off();

	pin_write(&g_cfg.open_out, GPIO_PIN_SET);

	g_state = GRIPPER_OPENING;

	timeout_reset();
}

static void action_close(void) {
	all_outputs_off();

	pin_write(&g_cfg.close_out, GPIO_PIN_SET);

	g_state = GRIPPER_CLOSING;

	timeout_reset();
}

static void action_up(void) {
	all_outputs_off();

	pin_write(&g_cfg.up_out, GPIO_PIN_SET);

	g_state = GRIPPER_MOVING_UP;

	timeout_reset();
}

static void action_down(void) {
	all_outputs_off();

	pin_write(&g_cfg.down_out, GPIO_PIN_SET);

	g_state = GRIPPER_MOVING_DOWN;

	timeout_reset();
}

/* ============================================================================
 * Public
 * ========================================================================== */

void Gripper_Init(const GripperConfig_t *config) {
	g_cfg = *config;

	all_outputs_off();

	g_state = GRIPPER_IDLE;
}

void Gripper_Stop(void) {
	all_outputs_off();

	g_state = GRIPPER_IDLE;
}

void Gripper_Command(GripperCommand_t cmd) {
	switch (cmd) {
		case GRIPPER_CMD_OPEN:
			action_open();
			break;

		case GRIPPER_CMD_CLOSE:
			action_close();
			break;

		case GRIPPER_CMD_UP:
			action_up();
			break;

		case GRIPPER_CMD_DOWN:
			action_down();
			break;

		case GRIPPER_CMD_PICK:
			action_close();
			g_state = GRIPPER_PICKING;
			break;

		case GRIPPER_CMD_PLACE:
			action_down();
			g_state = GRIPPER_PLACING;
			break;

		default:
			break;
	}
}

void Gripper_Update(void) {
	switch (g_state) {

		/* ============================================================= */
		/* OPEN */
		/* ============================================================= */

		case GRIPPER_OPENING:

			if (Gripper_IsOpened()) {
				Gripper_Stop();
				g_state = GRIPPER_DONE;
			} else if (timeout_expired()) {
				Gripper_Stop();
				g_state = GRIPPER_ERROR;
			}

			break;

			/* ============================================================= */
			/* CLOSE */
			/* ============================================================= */

		case GRIPPER_CLOSING:

			if (Gripper_IsClosed()) {
				Gripper_Stop();
				g_state = GRIPPER_DONE;
			} else if (timeout_expired()) {
				Gripper_Stop();
				g_state = GRIPPER_ERROR;
			}

			break;

			/* ============================================================= */
			/* UP */
			/* ============================================================= */

		case GRIPPER_MOVING_UP:

			if (Gripper_IsUp()) {
				Gripper_Stop();
				g_state = GRIPPER_DONE;
			} else if (timeout_expired()) {
				Gripper_Stop();
				g_state = GRIPPER_ERROR;
			}

			break;

			/* ============================================================= */
			/* DOWN */
			/* ============================================================= */

		case GRIPPER_MOVING_DOWN:

			if (Gripper_IsDown()) {
				Gripper_Stop();
				g_state = GRIPPER_DONE;
			} else if (timeout_expired()) {
				Gripper_Stop();
				g_state = GRIPPER_ERROR;
			}

			break;

			/* ============================================================= */
			/* PICK */
			/* ============================================================= */

		case GRIPPER_PICKING:

			if (Gripper_IsClosed()) {
				action_up();
			} else if (timeout_expired()) {
				Gripper_Stop();
				g_state = GRIPPER_ERROR;
			}

			break;

			/* ============================================================= */
			/* PLACE */
			/* ============================================================= */

		case GRIPPER_PLACING:

			if (Gripper_IsDown()) {
				action_open();
			} else if (timeout_expired()) {
				Gripper_Stop();
				g_state = GRIPPER_ERROR;
			}

			break;

		default:
			break;
	}

	/* ============================================================= */
	/* PICK FINISH */
	/* ============================================================= */

	if (g_state == GRIPPER_MOVING_UP && Gripper_IsUp()) {
		Gripper_Stop();
		g_state = GRIPPER_DONE;
	}

	/* ============================================================= */
	/* PLACE FINISH */
	/* ============================================================= */

	if (g_state == GRIPPER_OPENING && Gripper_IsOpened()) {
		Gripper_Stop();
		g_state = GRIPPER_DONE;
	}
}

bool Gripper_IsBusy(void) {
	return !(g_state == GRIPPER_IDLE || g_state == GRIPPER_DONE
				|| g_state == GRIPPER_ERROR);
}

bool Gripper_IsDone(void) {
	return g_state == GRIPPER_DONE;
}

bool Gripper_HasError(void) {
	return g_state == GRIPPER_ERROR;
}

GripperState_t Gripper_GetState(void) {
	return g_state;
}

/* ============================================================================
 * Sensors
 * ========================================================================== */

bool Gripper_IsOpened(void) {
	return pin_read(&g_cfg.reed_open) == GPIO_PIN_SET;
}

bool Gripper_IsClosed(void) {
	return pin_read(&g_cfg.reed_close) == GPIO_PIN_SET;
}

bool Gripper_IsUp(void) {
	return pin_read(&g_cfg.reed_up) == GPIO_PIN_SET;
}

bool Gripper_IsDown(void) {
	return pin_read(&g_cfg.reed_down) == GPIO_PIN_SET;
}
