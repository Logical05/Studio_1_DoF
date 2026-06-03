/*
 * aican.c
 *
 *  Created on: Jun 3, 2026
 *      Author: kitth
 */

#include "comm/aican.h"
#include "comm/charmander.h"
#include "app/safety.h"
#include "drivers/gripper.h"

/* --- Private Properties --- */
static FDCAN_HandleTypeDef *_p_hfdcan = NULL;
static uint32_t _last_master_activity_ms = 0;
static uint8_t _last_confirmed_relays = 0;

/* Internal helpers */
static void AICAN_ResetWatchdog(void);
static void AICAN_ApplyFailsafe(void);
static void AICAN_ParseMasterCommand(uint8_t *payload, uint8_t dlc);

void AICAN_Init(FDCAN_HandleTypeDef *hfdcan) {
	_p_hfdcan = hfdcan;
	_last_master_activity_ms = HAL_GetTick();

	FDCAN_FilterTypeDef sFilterConfig;

	/* Filter Setup: Capture Master Requests (0x210) & Master Heartbeats (0x600)
	 * Using Mask Mode to trap specific ranges */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = MAKE_CAN_ID(AICAN_FUNC_CMD_REQ, AICAN_NODE_ID); // 0x210
	sFilterConfig.FilterID2 = 0x7F0; // Allows 0x210 and easily captures broadcast patterns

	if (HAL_FDCAN_ConfigFilter(_p_hfdcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Secondary Filter for the Master Broadcast Heartbeat (0x600) */
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterID1 = MAKE_CAN_ID(AICAN_FUNC_HEARTBEAT,
			AICAN_MASTER_ID); // 0x600
	sFilterConfig.FilterID2 = 0x7FF; // Absolute precise match

	if (HAL_FDCAN_ConfigFilter(_p_hfdcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Start Bus Operations */
	if (HAL_FDCAN_Start(_p_hfdcan) != HAL_OK) {
		Error_Handler();
	}

	/* Enable Interrupt Handlers for Rx FIFO 0 */
	if (HAL_FDCAN_ActivateNotification(_p_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		Error_Handler();
	}
}

void AICAN_Update(void) {
	/* Process Failsafe / Hardware Watchdog Boundaries */
	uint32_t current_time = HAL_GetTick();

	if ((current_time - _last_master_activity_ms) >= AICAN_WATCHDOG_TIMEOUT_MS) {
		AICAN_ApplyFailsafe();
	}
}

void AICAN_ProcessInterrupt(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data)
				== HAL_OK) {
			uint32_t received_id = rx_header.Identifier;
			uint8_t dlc_bytes = (rx_header.DataLength >> 16) & 0x0F; // Extract byte length from FDCAN mapping

			/* Handle Global Broadcast Heartbeat (0x600) */
			if (received_id
					== MAKE_CAN_ID(AICAN_FUNC_HEARTBEAT, AICAN_MASTER_ID)) {
				if (dlc_bytes >= 1) {
					if (rx_data[0] == 0x05) {
						AICAN_ResetWatchdog(); // Master Operational
					} else if (rx_data[0] == 0x00) {
						AICAN_ApplyFailsafe();  // Master Stopped Command
					}
				}
			}
			/* Handle Direct Commands Directed to Us (0x210) */
			else if (received_id
					== MAKE_CAN_ID(AICAN_FUNC_CMD_REQ, AICAN_NODE_ID)) {
				AICAN_ResetWatchdog(); // Commands inherently reset the timeout loop
				AICAN_ParseMasterCommand(rx_data, dlc_bytes);
			}
		}
	}
}

void AICAN_SendReadResponse(uint8_t sensor_mask) {
	if (_p_hfdcan == NULL)
		return;

	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[3];

	tx_header.Identifier = MAKE_CAN_ID(AICAN_FUNC_CMD_RESP, AICAN_NODE_ID); // 0x310
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = FDCAN_DLC_BYTES_3;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	tx_header.MessageMarker = 0;

	tx_data[0] = AICAN_CMD_READ_RESP; // 0x21
	tx_data[1] = AICAN_OPTO_BANK;     // 0x10
	tx_data[2] = sensor_mask;         // Bitwise limit flags

	HAL_FDCAN_AddMessageToTxFifoQ(_p_hfdcan, &tx_header, tx_data);
}

/* --- Private Implementation Details --- */

static void AICAN_ResetWatchdog(void) {
	_last_master_activity_ms = HAL_GetTick();
}

static void AICAN_ApplyFailsafe(void) {
	/* Force gripper configurations to dead state */
	charmander.gripper_vertical = CHARMANDER_VERTICAL_IDLE;
	charmander.gripper_jaw = CHARMANDER_JAW_IDLE;
	charmander.gripper_seq = CHARMANDER_SEQ_NONE;

	/* Optional safety interface injection */
	// Charmander_SetEmergency(1);
}

static void AICAN_ParseMasterCommand(uint8_t *payload, uint8_t dlc) {
	if (dlc < 2)
		return;

	uint8_t instruction = payload[0];
	uint8_t target_bank = payload[1];

	/* Case A: Master requests a Relay Output modification */
	if (instruction == AICAN_CMD_WRITE_REQ && target_bank == AICAN_RELAY_BANK) {
		if (dlc < 3)
			return;
		uint8_t mask = payload[2];

		// Route directly to your hardware drivers / Charmander state tracker
		if (mask & (1 << 0))
			charmander.gripper_vertical = CHARMANDER_VERTICAL_UP;   // Relay 0
		if (mask & (1 << 1))
			charmander.gripper_vertical = CHARMANDER_VERTICAL_DOWN; // Relay 1
		if (mask & (1 << 2))
			charmander.gripper_jaw = CHARMANDER_JAW_CLOSE;     // Relay 2
		if (mask & (1 << 3))
			charmander.gripper_jaw = CHARMANDER_JAW_OPEN;      // Relay 3

		_last_confirmed_relays = mask;

		/* Prepare Write Acknowledge back to Master (0x310) */
		FDCAN_TxHeaderTypeDef ack_header;
		uint8_t ack_data[3];

		ack_header.Identifier = MAKE_CAN_ID(AICAN_FUNC_CMD_RESP, AICAN_NODE_ID); // 0x310
		ack_header.IdType = FDCAN_STANDARD_ID;
		ack_header.TxFrameType = FDCAN_DATA_FRAME;
		ack_header.DataLength = FDCAN_DLC_BYTES_3;
		ack_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		ack_header.BitRateSwitch = FDCAN_BRS_OFF;
		ack_header.FDFormat = FDCAN_CLASSIC_CAN;
		ack_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		ack_header.MessageMarker = 0;

		ack_data[0] = AICAN_CMD_WRITE_ACK; // 0x11
		ack_data[1] = AICAN_RELAY_BANK;    // 0x00
		ack_data[2] = _last_confirmed_relays;

		HAL_FDCAN_AddMessageToTxFifoQ(_p_hfdcan, &ack_header, ack_data);
	}
	/* Case B: Master requests an Opto-Isolator Read transaction */
	else if (instruction == AICAN_CMD_READ_REQ && target_bank == AICAN_OPTO_BANK) {
		uint8_t sensors = 0;
		if (Gripper_IsUp())
			sensors |= (1 << 0);
		if (Gripper_IsDown())
			sensors |= (1 << 1);
		if (Gripper_IsClosed())
			sensors |= (1 << 2);

		AICAN_SendReadResponse(sensors);
	}
}

