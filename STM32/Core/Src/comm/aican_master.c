/*
 * aican_master.c
 * CAN master — commands gripper node 0x10 via relay/opto protocol.
 */

#include "comm/aican_master.h"
#include "config/pinmap.h"
#include "drivers/gripper.h"

/* ── Global debug struct — watch "can" in Live Expressions ─────── */
AICAN_Debug_t can = { 0 };

/* ── Private state ──────────────────────────────────────────────── */
static FDCAN_HandleTypeDef *s_hfdcan = NULL;
static uint32_t s_lastHB = 0;
static uint32_t s_lastOpto = 0;
static uint32_t s_lastRelay = 0;
static uint8_t s_currentRelay = AICAN_RELAY_NONE;
#define AICAN_RELAY_RESEND_MS  100

/* ── Build TX header (avoids repeating boilerplate) ────────────── */
static FDCAN_TxHeaderTypeDef MakeTxHeader(uint32_t id, uint32_t dlc) {
	FDCAN_TxHeaderTypeDef h = { 0 };
	h.Identifier = id;
	h.IdType = FDCAN_STANDARD_ID;
	h.TxFrameType = FDCAN_DATA_FRAME;
	h.DataLength = dlc;
	h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	h.BitRateSwitch = FDCAN_BRS_OFF;
	h.FDFormat = FDCAN_CLASSIC_CAN;
	h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	h.MessageMarker = 0;
	return h;
}

/* ── Read GPIO reeds directly from pinmap ───────────────────────── */
static void ReadGPIO(void) {
	can.gpio_isUp =
			(HAL_GPIO_ReadPin(GRIPPER_REED_UP_PORT, GRIPPER_REED_UP_PIN) != GPIO_PIN_RESET) ?
					1u : 0u;
	can.gpio_isDown =
			(HAL_GPIO_ReadPin(GRIPPER_REED_DOWN_PORT, GRIPPER_REED_DOWN_PIN) != GPIO_PIN_RESET) ?
					1u : 0u;
	can.gpio_isClosed =
			(HAL_GPIO_ReadPin(GRIPPER_REED_CLOSE_PORT, GRIPPER_REED_CLOSE_PIN) != GPIO_PIN_RESET) ?
					1u : 0u;
}

/* ── Pack everything into one statusWord for quick Live Expression ─ */
static void UpdateStatusWord(void) {
	can.statusWord = ((uint32_t) (can.errorCount & 0xFF) << 24)
			| ((uint32_t) (can.lastRelayCmd & 0xFF) << 16)
			| ((uint32_t) (can.actualRelay & 0xFF) << 8)
			| ((uint32_t) (can.sensorMismatch & 1) << 7)
			| ((uint32_t) (can.ackReceived & 1) << 6)
			| ((uint32_t) (can.gpio_isClosed & 1) << 5)
			| ((uint32_t) (can.gpio_isDown & 1) << 4)
			| ((uint32_t) (can.gpio_isUp & 1) << 3)
			| ((uint32_t) (can.opto_jaw & 1) << 2)
			| ((uint32_t) (can.opto_down & 1) << 1)
			| ((uint32_t) (can.opto_up & 1) << 0);
}

/* ── Send heartbeat 0x600 — payload MUST be 0x05 ───────────────── */
static void SendHeartbeat(void) {
	uint8_t d[1] = { AICAN_HB_OPERATIONAL };
	FDCAN_TxHeaderTypeDef h = MakeTxHeader(AICAN_ID_HEARTBEAT, FDCAN_DLC_BYTES_1);
	HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &h, d);
	can.heartbeatCount++;
	can.txCount++;
}

/* ── Request opto read from node ────────────────────────────────── */
static void RequestOptoRead(void) {
	uint8_t d[2] = { AICAN_INSTR_READ_REQ, AICAN_BANK_OPTO };
	FDCAN_TxHeaderTypeDef h = MakeTxHeader(AICAN_ID_CMD_REQ, FDCAN_DLC_BYTES_2);
	HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &h, d);
	can.txCount++;
}

/* ═══════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

void AICAN_Master_Init(FDCAN_HandleTypeDef *hfdcan) {
	s_hfdcan = hfdcan;
	s_lastHB = HAL_GetTick();
	s_lastOpto = HAL_GetTick();

	/* Accept 0x110 (opto broadcast) and 0x310 (cmd response) only */
	FDCAN_FilterTypeDef f = { 0 };
	f.IdType = FDCAN_STANDARD_ID;
	f.FilterType = FDCAN_FILTER_DUAL;
	f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	f.FilterIndex = 0;
	f.FilterID1 = 0x110;
	f.FilterID2 = 0x310;
	HAL_FDCAN_ConfigFilter(hfdcan, &f);

	/* Reject everything else — stops other lab nodes flooding the FIFO */
	HAL_FDCAN_ConfigGlobalFilter(hfdcan,
	FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(hfdcan);

	AICAN_Master_SendRelays(AICAN_RELAY_NONE); /* safe start */
}

/* Called from App_Update() every loop */
void AICAN_Master_Update(void) {
	if (s_hfdcan == NULL) return;
	uint32_t now = HAL_GetTick();

	if ((now - s_lastHB) >= AICAN_HEARTBEAT_MS) {
		SendHeartbeat();
		s_lastHB = now;
	}

	if ((now - s_lastOpto) >= AICAN_OPTO_POLL_MS) {
		RequestOptoRead();
		s_lastOpto = now;
	}

	/* Always refresh GPIO and summary */
	ReadGPIO();

	can.sensorMismatch =
			(can.opto_up != can.gpio_isUp || can.opto_down != can.gpio_isDown
				|| can.opto_jaw != can.gpio_isClosed) ? 1u : 0u;

	UpdateStatusWord();

	AICAN_Master_SyncGripper();
}

/* Send relay bitmask to node — rejects conflicting bits */
void AICAN_Master_SendRelays(uint8_t mask) {
	if (s_hfdcan == NULL) return;

	/* Refuse UP+DOWN or OPEN+CLOSE at the same time */
	if (((mask & AICAN_RELAY_UP) && (mask & AICAN_RELAY_DOWN)) || ((mask
			& AICAN_RELAY_OPEN)
																	&& (mask & AICAN_RELAY_CLOSE))) {
		can.errorCount++;
		return;
	}

	uint8_t d[3] = { AICAN_INSTR_WRITE_REQ, AICAN_BANK_RELAY, mask };
	FDCAN_TxHeaderTypeDef h = MakeTxHeader(AICAN_ID_CMD_REQ, FDCAN_DLC_BYTES_3);
	HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &h, d);

	can.lastRelayCmd = mask;
	can.ackReceived = 0;
	can.txCount++;
}

/* Forward from HAL_FDCAN_RxFifo0Callback */
void AICAN_Master_RxCallback(FDCAN_HandleTypeDef *hfdcan) {
	FDCAN_RxHeaderTypeDef rx;
	uint8_t d[8] = { 0 };

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) != HAL_OK) return;

	can.lastRxId = rx.Identifier;
	can.rxB0 = d[0];
	can.rxB1 = d[1];
	can.rxB2 = d[2];
	can.rxCount++;

	if (rx.Identifier == AICAN_ID_CMD_RESP) { /* 0x310 */
		if (d[0] == AICAN_INSTR_WRITE_ACK) { /* relay write confirmed */
			can.actualRelay = d[2];
			can.ackReceived = 1;
			if (d[2] != can.lastRelayCmd) can.errorCount++;

		} else if (d[0] == AICAN_INSTR_READ_RESP) { /* opto data arrived */
			can.optoRaw = d[2];
			can.opto_up = (d[2] & AICAN_OPTO_UP) ? 1u : 0u;
			can.opto_down = (d[2] & AICAN_OPTO_DOWN) ? 1u : 0u;
			can.opto_jaw = (d[2] & AICAN_OPTO_JAW) ? 1u : 0u;

		} else {
			can.errorCount++;
		}
	}
}

static GripperState_t s_lastGripperState = GRIPPER_IDLE;

void AICAN_Master_SyncGripper(void) {
	uint8_t desired = AICAN_RELAY_NONE;

	switch (gripper.state) {
		case GRIPPER_OPENING:
			desired = AICAN_RELAY_OPEN;
			break;
		case GRIPPER_CLOSING:
			desired = AICAN_RELAY_CLOSE;
			break;
		case GRIPPER_MOVING_UP:
			desired = AICAN_RELAY_UP;
			break;
		case GRIPPER_MOVING_DOWN:
			desired = AICAN_RELAY_DOWN;
			break;
		case GRIPPER_PICK_DOWN:
			desired = AICAN_RELAY_DOWN;
			break;
		case GRIPPER_PICK_CLOSE:
			desired = AICAN_RELAY_CLOSE;
			break;
		case GRIPPER_PICK_UP:
			desired = AICAN_RELAY_UP;
			break;
		case GRIPPER_PLACE_DOWN:
			desired = AICAN_RELAY_DOWN;
			break;
		case GRIPPER_PLACE_OPEN:
			desired = AICAN_RELAY_OPEN;
			break;
		case GRIPPER_PLACE_UP:
			desired = AICAN_RELAY_UP;
			break;
		default:
			desired = AICAN_RELAY_NONE;
			break;
	}

	uint32_t now = HAL_GetTick();
	if (desired != s_currentRelay || (now - s_lastRelay) >= AICAN_RELAY_RESEND_MS) {
		s_currentRelay = desired;
		s_lastRelay = now;
		AICAN_Master_SendRelays(desired);
	}
}
