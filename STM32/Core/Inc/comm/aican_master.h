/*
 * aican_master.h
 * CAN master library — commands gripper node (ID 0x10) via relay bits.
 *
 * LIVE EXPRESSION: add "can" to watch everything at once.
 *
 *  can.statusWord   ← one hex number, decode below
 *  can.errorCount   ← should stay 0
 *  can.ackReceived  ← should flip 1 after every send
 *
 * statusWord bit map:
 *   [31:24] errorCount (low byte)
 *   [23:16] lastRelayCmd   — what we sent
 *   [15: 8] actualRelay    — what node confirmed
 *   [    7] sensorMismatch — CAN opto != GPIO reed
 *   [    6] ackReceived
 *   [    5] gpio_isClosed  PB15
 *   [    4] gpio_isDown    PB1
 *   [    3] gpio_isUp      PB2
 *   [    2] opto_jaw       (from node)
 *   [    1] opto_down      (from node)
 *   [    0] opto_up        (from node)
 */

#ifndef INC_COMM_AICAN_MASTER_H_
#define INC_COMM_AICAN_MASTER_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ── CAN ID constants ───────────────────────────────────────────── */
#define AICAN_MAKE_ID(func, node)   (((func & 0x07) << 8) | (node & 0xFF))

#define AICAN_NODE_ID               0x10
#define AICAN_MASTER_ID             0x00

#define AICAN_ID_CMD_REQ            AICAN_MAKE_ID(0x2, 0x10)  /* 0x210 master->node */
#define AICAN_ID_CMD_RESP           AICAN_MAKE_ID(0x3, 0x10)  /* 0x310 node->master */
#define AICAN_ID_HEARTBEAT          AICAN_MAKE_ID(0x6, 0x00)  /* 0x600 broadcast    */

/* ── Instruction / bank bytes ───────────────────────────────────── */
#define AICAN_INSTR_WRITE_REQ       0x10
#define AICAN_INSTR_WRITE_ACK       0x11
#define AICAN_INSTR_READ_REQ        0x20
#define AICAN_INSTR_READ_RESP       0x21
#define AICAN_BANK_RELAY            0x00
#define AICAN_BANK_OPTO             0x10
#define AICAN_HB_OPERATIONAL        0x05   /* MUST be 0x05 — anything else ignored by node */

/* ── Relay bit map (Byte 2 of write command) ────────────────────── *
 *  Bit 0 = Relay 0 = UP   solenoid  (GRIPPER_UP_OUT   PB7 )       *
 *  Bit 1 = Relay 1 = DOWN solenoid  (GRIPPER_DOWN_OUT PA15)       *
 *  Bit 2 = Relay 2 = CLOSE solenoid (GRIPPER_CLOSE_OUT PB13)      *
 *  Bit 3 = Relay 3 = OPEN  solenoid (GRIPPER_OPEN_OUT  PB12)      *
 * ─────────────────────────────────────────────────────────────── */
#define AICAN_RELAY_NONE            0x00
#define AICAN_RELAY_UP              (1 << 0)
#define AICAN_RELAY_DOWN            (1 << 1)
#define AICAN_RELAY_CLOSE           (1 << 2)
#define AICAN_RELAY_OPEN            (1 << 3)

/* ── Opto bit map (Byte 2 of read response) ─────────────────────── *
 *  Bit 0 = UP   reed (GRIPPER_REED_UP    PB2 )                     *
 *  Bit 1 = DOWN reed (GRIPPER_REED_DOWN  PB1 )                     *
 *  Bit 2 = JAW  reed (GRIPPER_REED_CLOSE PB15)                     *
 * ─────────────────────────────────────────────────────────────── */
#define AICAN_OPTO_UP               (1 << 0)
#define AICAN_OPTO_DOWN             (1 << 1)
#define AICAN_OPTO_JAW              (1 << 2)

/* ── Timing ─────────────────────────────────────────────────────── */
#define AICAN_HEARTBEAT_MS          500    /* node watchdog = 2000ms, 500ms = safe */
#define AICAN_OPTO_POLL_MS          200

/* ── Debug struct — add "can" to Live Expressions ───────────────── */
typedef struct {
	uint32_t statusWord; /* one-number summary, see bit map in header  */

	/* relay */
	uint8_t lastRelayCmd; /* mask we last sent                          */
	uint8_t actualRelay; /* mask node confirmed                        */
	uint8_t ackReceived; /* 1 = node ACKed last write                  */

	/* opto from CAN (updated every 200 ms) */
	uint8_t optoRaw; /* raw bitmask from node                      */
	uint8_t opto_up;
	uint8_t opto_down;
	uint8_t opto_jaw;

	/* GPIO reed switches — direct read, always live, no CAN needed */
	uint8_t gpio_isUp; /* PB2  REED_UP                               */
	uint8_t gpio_isDown; /* PB1  REED_DOWN                             */
	uint8_t gpio_isClosed; /* PB15 REED_CLOSE                            */

	/* health */
	uint8_t sensorMismatch; /* 1 = CAN opto disagrees with GPIO           */
	uint32_t heartbeatCount; /* should increment ~2/sec                    */
	uint32_t txCount;
	uint32_t rxCount;
	uint32_t errorCount;

	/* last raw frame (protocol debug) */
	uint32_t lastRxId;
	uint8_t rxB0, rxB1, rxB2;

} AICAN_Debug_t;

extern AICAN_Debug_t can; /* <── add "can" to Live Expressions */

/* ── Public API ─────────────────────────────────────────────────── */
void AICAN_Master_Init(FDCAN_HandleTypeDef *hfdcan);
void AICAN_Master_Update(void); /* call from App_Update  */
void AICAN_Master_SendRelays(uint8_t mask); /* send relay bitmask    */
void AICAN_Master_RxCallback(FDCAN_HandleTypeDef *hfdcan); /* from HAL CB */
void AICAN_Master_SyncGripper(void);

#endif /* INC_COMM_AICAN_MASTER_H_ */
