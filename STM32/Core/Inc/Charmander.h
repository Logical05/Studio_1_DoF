/**
 * @file    charmander.h
 * @brief   Charmander — Modbus RTU Command Bridge for STM32G474RE
 *
 * Handles:
 *   - FC06 WRITE frames  (PC → Robot, registers 0x00–0x25)
 *   - FC03 READ  frames  (PC → Robot query; robot replies with registers 0x00, 0x26–0x31)
 *   - Heartbeat exchange (Robot sends YA=22881, PC answers HI=18537 via write to 0x00)
 */
/*change*/

#ifndef CHARMANDER_H
#define CHARMANDER_H

#include <stdint.h>

typedef enum {
	CHARMANDER_HB_WAITING, CHARMANDER_HB_ALIVE, CHARMANDER_HB_DEAD
} CharmanderHeartbeatState_t;

typedef enum {
	CHARMANDER_MODE_IDLE,
	CHARMANDER_MODE_HOME,
	CHARMANDER_MODE_JOG,
	CHARMANDER_MODE_AUTO,
	CHARMANDER_MODE_P2P,
	CHARMANDER_MODE_SET_HOME,
	CHARMANDER_MODE_TEST
} CharmanderMode_t;

typedef enum {
	CHARMANDER_VERTICAL_IDLE, CHARMANDER_VERTICAL_UP, CHARMANDER_VERTICAL_DOWN
} CharmanderVerticalState_t;

typedef enum {
	CHARMANDER_JAW_IDLE, CHARMANDER_JAW_OPEN, CHARMANDER_JAW_CLOSE
} CharmanderJawState_t;

typedef enum {
	CHARMANDER_SEQ_NONE, CHARMANDER_SEQ_PICK, CHARMANDER_SEQ_PLACE
} CharmanderSequence_t;

typedef enum {
	CHARMANDER_DISABLE, CHARMANDER_ENABLE
} CharmanderEnable_t;

typedef enum {
	CHARMANDER_JOG_NONE, CHARMANDER_JOG_CW, CHARMANDER_JOG_CCW
} CharmanderJogDir_t;

typedef enum {
	CHARMANDER_TEST_PRECISION, CHARMANDER_TEST_PERFORMANCE
} CharmanderTestType_t;

typedef enum {
	CHARMANDER_UNIT_DEGREE, CHARMANDER_UNIT_INDEX
} CharmanderUnit_t;

typedef enum {
	CHARMANDER_STOP_RUNNING, CHARMANDER_STOP_ACTIVE
} CharmanderSoftStop_t;

typedef enum {
	CHARMANDER_SENSOR_IDLE, CHARMANDER_SENSOR_UP, CHARMANDER_SENSOR_DOWN
} CharmanderSensorVertical_t;

typedef enum {
	CHARMANDER_SENSOR_OPEN, CHARMANDER_SENSOR_CLOSED
} CharmanderSensorJaw_t;

typedef enum {
	CHARMANDER_TASK_IDLE,
	CHARMANDER_TASK_HOMING,
	CHARMANDER_TASK_GO_PICK,
	CHARMANDER_TASK_GO_PLACE,
	CHARMANDER_TASK_GO_POINT
} CharmanderTask_t;

typedef enum {
	CHARMANDER_EMERGENCY_NORMAL, CHARMANDER_EMERGENCY_ACTIVE
} CharmanderEmergency_t;

typedef enum {
	CHARMANDER_TX_IDLE, CHARMANDER_TX_SENT
} CharmanderTxStatus_t;

typedef enum {
	CHARMANDER_RX_NO_REPLY, CHARMANDER_RX_GOT_HI, CHARMANDER_RX_BAD_VALUE
} CharmanderRxStatus_t;

/* ─── Configuration ─────────────────────────────────────────────── */
#define CHARMANDER_SLAVE_ADDR        21      /* Default slave address (0x15) */
#define CHARMANDER_FC_WRITE          0x06    /* Write Single Register          */
#define CHARMANDER_FC_READ           0x03    /* Read Holding Registers         */
#define CHARMANDER_WRITE_FRAME_SIZE  8       /* FC06 frame is always 8 bytes   */
#define CHARMANDER_READ_REQ_SIZE     8       /* FC03 request is always 8 bytes */
#define CHARMANDER_RX_BUF_SIZE       64      /* Ring buffer size (power of 2)  */

/* ─── Heartbeat magic values ────────────────────────────────────── */
#define CHARMANDER_HB_YA             22881   /* 0x5941 "YA" — Robot → PC (sent on read 0x00) */
#define CHARMANDER_HB_HI             18537   /* 0x4869 "HI" — PC → Robot (written to 0x00)   */

/* ─── Heartbeat timing ──────────────────────────────────────────── */
#define CHARMANDER_HB_INTERVAL_MS    500     /* Send "YA" every 500 ms                        */
#define CHARMANDER_HB_TIMEOUT_MS     2000    /* Mark DEAD if no "HI" reply within 2 s         */

/* ─── READ block definition ─────────────────────────────────────── */
/*  The PC reads registers 0x00 and 0x26–0x31 in a single FC03 block.  */
#define CHARMANDER_READ_START_ADDR   0x0000
#define CHARMANDER_READ_COUNT        50      /* Covers 0x00 … 0x31 (50 registers) */

/* ═══════════════════════════════════════════════════════════════════
 *  STATE STRUCT
 *  All fields are updated by Charmander_Process() as frames arrive.
 *  Read-only from application code.
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct {

	/* ────────────────────────────────────────────────────────────
	 *  WRITE side  (PC → Robot, registers 0x00–0x25)
	 * ──────────────────────────────────────────────────────────── */

	/* 0x00  Heartbeat */
	CharmanderHeartbeatState_t heartbeat; /* "ALIVE" | "WAITING" | "DEAD" */

	/* 0x01  Operating mode */
	CharmanderMode_t mode; /* "HOME" | "JOG" | "AUTO" | "SET_HOME" | "TEST" | "IDLE" */
	uint16_t mode_raw;

	/* 0x02  Manual gripper */
	CharmanderVerticalState_t gripper_vertical; /* "UP" | "DOWN" | "IDLE" */
	CharmanderJawState_t gripper_jaw; /* "OPEN" | "CLOSE" | "IDLE" */
	uint16_t gripper_cmd_raw;

	/* 0x03  Gripper sequence */
	CharmanderSequence_t gripper_seq; /* "PICK" | "PLACE" | "NONE" */
	uint16_t gripper_seq_raw;

	/* 0x04  Gripper auto enable */
	CharmanderEnable_t gripper_auto; /* "ENABLED" | "DISABLED" */
	uint16_t gripper_auto_raw;

	/* 0x05  Jog */
	CharmanderJogDir_t jog_dir; /* "CCW" | "CW" | "NONE" */
	int16_t jog_degrees;

	/* 0x06  Test type */
	CharmanderTestType_t test_type; /* "PRECISION" | "PERFORMANCE" */

	/* 0x07  Performance test – velocity */
	int16_t perf_velocity;

	/* 0x08  Performance test – acceleration */
	int16_t perf_acceleration;

	/* 0x09  Precision test – initial position */
	int16_t prec_init_pos;

	/* 0x10  Precision test – final position */
	int16_t prec_final_pos;

	/* 0x11  Precision test – repeat count (sign encodes unit) */
	int16_t prec_repeat_count;
	CharmanderUnit_t prec_unit; /* "DEGREE" | "INDEX" */

	/* 0x12–0x21  Pick & place sequence slots (16 slots) */
	int16_t pp_slots[16];

	/* 0x22  Pick & place pair count */
	uint16_t pp_pair_count;

	/* 0x23  Point-to-point unit */
	CharmanderUnit_t p2p_unit; /* "DEGREE" | "INDEX" */

	/* 0x24  Point-to-point target */
	int16_t p2p_target;

	/* 0x25  Soft stop */
	CharmanderSoftStop_t soft_stop; /* "STOP" | "RUNNING" */

	/* ────────────────────────────────────────────────────────────
	 *  READ side  (Robot → PC, registers 0x00, 0x26–0x31)
	 *  Application code fills these; Charmander_BuildReadReply()
	 *  serialises them into the FC03 response frame.
	 * ──────────────────────────────────────────────────────────── */

	/* 0x26  Lead / reed sensors (bit-field) */
	uint16_t sensor_raw; /* raw register value         */
	CharmanderSensorVertical_t sensor_vertical; /* "UP" | "DOWN" | "IDLE"     */
	CharmanderSensorJaw_t sensor_jaw; /* "OPEN" | "CLOSED"          */

	/* 0x27  Current robot task (bit-field) */
	uint16_t task_raw;
	CharmanderTask_t task; /* "HOMING" | "GO_PICK" | "GO_PLACE" | "GO_POINT" | "IDLE" */

	/* 0x28  Real position  (register value = real × 10) */
	int16_t position_raw; /* raw register (÷10 for display) */
	float position_real; /* decoded, in user units         */

	/* 0x29  Real velocity  (register value = real × 10) */
	int16_t velocity_raw;
	float velocity_real;

	/* 0x30  Real acceleration  (register value = real × 10) */
	int16_t accel_raw;
	float accel_real;

	/* 0x31  Emergency / safety state */
	uint16_t emergency_raw;
	CharmanderEmergency_t emergency; /* "ACTIVE" | "NORMAL" */

	/* ────────────────────────────────────────────────────────────
	 *  Heartbeat timing (managed internally by Charmander_Tick)
	 * ──────────────────────────────────────────────────────────── */
	uint32_t hb_last_sent_ms; /* HAL_GetTick() when YA was last transmitted */
	uint32_t hb_last_alive_ms; /* HAL_GetTick() when HI was last received    */

	/* ────────────────────────────────────────────────────────────
	 *  Heartbeat counters & TX status
	 * ──────────────────────────────────────────────────────────── */
	CharmanderTxStatus_t hb_tx_status; /* "SENT" | "IDLE"  — flips each send  */
	CharmanderRxStatus_t hb_rx_status; /* "GOT_HI" | "NO_REPLY" — current state */

	/* ────────────────────────────────────────────────────────────
	 *  Debug / diagnostics
	 * ──────────────────────────────────────────────────────────── */
	uint16_t last_reg_addr;
	uint16_t last_reg_value;
	uint32_t frame_count; /* total valid frames processed  */
	uint32_t crc_error_count; /* frames dropped due to CRC     */
	uint32_t read_req_count; /* FC03 read requests handled    */
	uint8_t tx_reply_buf[105];

} Charmander_t;

/* ═══════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise all state to safe defaults. Call once before the main loop.
 */
void Charmander_Init(void);

/**
 * @brief  Push one received byte into the internal ring buffer.
 *         Call from HAL_UART_RxCpltCallback (interrupt context – must be fast).
 */
void Charmander_Feed(uint8_t byte);

/**
 * @brief  Parse accumulated bytes into complete Modbus frames.
 *         Call repeatedly from the main loop.
 */
void Charmander_Process(void);

/**
 * @brief  Call from the main loop every iteration (no delay needed).
 *         Internally manages the heartbeat interval (sends "YA" every
 *         CHARMANDER_HB_INTERVAL_MS) and marks the link "DEAD" if no
 *         "HI" reply arrives within CHARMANDER_HB_TIMEOUT_MS.
 *         Replaces manual calls to Charmander_SendHeartbeat() in main.c.
 */
void Charmander_Tick(void);

/**
 * @brief  Build and transmit an FC03 Read-Holding-Registers reply containing
 *         registers 0x00 and 0x26–0x31.  The application must update the READ-side
 *         fields in charmander (sensor_raw, task_raw, position_raw, etc.) before
 *         calling this function or immediately after each motion cycle.
 *         Called automatically by Charmander_Process() when an FC03 request arrives.
 */
void Charmander_BuildReadReply(uint16_t start_addr, uint16_t reg_count);

void Charmander_SetSensors(uint16_t sensor_bits);

void Charmander_SetTask(uint16_t task_bits);

void Charmander_SetMotion(float pos, float vel, float acc);

void Charmander_SetEmergency(uint8_t active);

/* ─── Global instance (defined in charmander.c) ─────────────────── */
extern Charmander_t charmander;

#endif /* CHARMANDER_H */
