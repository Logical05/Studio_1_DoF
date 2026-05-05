/**
 * @file    charmander.h
 * @brief   Charmander — Modbus RTU Command Bridge for STM32G474RE
 *
 * Handles:
 *   - FC06 WRITE frames  (PC → Robot, registers 0x00–0x25)
 *   - FC03 READ  frames  (PC → Robot query; robot replies with registers 0x00, 0x26–0x31)
 *   - Heartbeat exchange (Robot sends YA=22881, PC answers HI=18537 via write to 0x00)
 */

#ifndef CHARMANDER_H
#define CHARMANDER_H

#include <stdint.h>

/* ─── Configuration ─────────────────────────────────────────────── */
#define CHARMANDER_SLAVE_ADDR        21      /* Default slave address (0x15) */
#define CHARMANDER_FC_WRITE          0x06    /* Write Single Register          */
#define CHARMANDER_FC_READ           0x03    /* Read Holding Registers         */
#define CHARMANDER_WRITE_FRAME_SIZE  8       /* FC06 frame is always 8 bytes   */
#define CHARMANDER_READ_REQ_SIZE     8       /* FC03 request is always 8 bytes */
#define CHARMANDER_RX_BUF_SIZE       64      /* Ring buffer size (power of 2)  */
#define CHARMANDER_LABEL_LEN         10

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
	char heartbeat[CHARMANDER_LABEL_LEN]; /* "ALIVE" | "WAITING" | "DEAD" */

	/* 0x01  Operating mode */
	char mode[CHARMANDER_LABEL_LEN]; /* "HOME" | "JOG" | "AUTO" | "SET_HOME" | "TEST" | "IDLE" */
	uint16_t mode_raw;

	/* 0x02  Manual gripper */
	char gripper_vertical[CHARMANDER_LABEL_LEN]; /* "UP" | "DOWN" | "IDLE" */
	char gripper_jaw[CHARMANDER_LABEL_LEN]; /* "OPEN" | "CLOSE" | "IDLE" */
	char gripper_jaw_last[CHARMANDER_LABEL_LEN]; /* last non-IDLE jaw command */
	uint16_t gripper_cmd_raw;

	/* 0x03  Gripper sequence */
	char gripper_seq[CHARMANDER_LABEL_LEN]; /* "PICK" | "PLACE" | "NONE" */
	uint16_t gripper_seq_raw;

	/* 0x04  Gripper auto enable */
	char gripper_auto[CHARMANDER_LABEL_LEN]; /* "ENABLED" | "DISABLED" */
	uint16_t gripper_auto_raw;

	/* 0x05  Jog */
	char jog_dir[CHARMANDER_LABEL_LEN]; /* "CCW" | "CW" | "NONE" */
	int16_t jog_degrees;

	/* 0x06  Test type */
	char test_type[CHARMANDER_LABEL_LEN]; /* "PRECISION" | "PERFORMANCE" */

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
	char prec_unit[CHARMANDER_LABEL_LEN]; /* "DEGREE" | "INDEX" */

	/* 0x12–0x21  Pick & place sequence slots (16 slots) */
	int16_t pp_slots[16];

	/* 0x22  Pick & place pair count */
	uint16_t pp_pair_count;

	/* 0x23  Point-to-point unit */
	char p2p_unit[CHARMANDER_LABEL_LEN]; /* "DEGREE" | "INDEX" */

	/* 0x24  Point-to-point target */
	int16_t p2p_target;

	/* 0x25  Soft stop */
	char soft_stop[CHARMANDER_LABEL_LEN]; /* "STOP" | "RUNNING" */

	/* ────────────────────────────────────────────────────────────
	 *  READ side  (Robot → PC, registers 0x00, 0x26–0x31)
	 *  Application code fills these; Charmander_BuildReadReply()
	 *  serialises them into the FC03 response frame.
	 * ──────────────────────────────────────────────────────────── */

	/* 0x26  Lead / reed sensors (bit-field) */
	uint16_t sensor_raw; /* raw register value         */
	char sensor_vertical[CHARMANDER_LABEL_LEN]; /* "UP" | "DOWN" | "IDLE"     */
	char sensor_jaw[CHARMANDER_LABEL_LEN]; /* "OPEN" | "CLOSED"          */

	/* 0x27  Current robot task (bit-field) */
	uint16_t task_raw;
	char task[CHARMANDER_LABEL_LEN]; /* "HOMING" | "GO_PICK" | "GO_PLACE" | "GO_POINT" | "IDLE" */

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
	char emergency[CHARMANDER_LABEL_LEN]; /* "ACTIVE" | "NORMAL" */

	/* ────────────────────────────────────────────────────────────
	 *  Heartbeat timing (managed internally by Charmander_Tick)
	 * ──────────────────────────────────────────────────────────── */
	uint32_t hb_last_sent_ms; /* HAL_GetTick() when YA was last transmitted */
	uint32_t hb_last_alive_ms; /* HAL_GetTick() when HI was last received    */

	/* ────────────────────────────────────────────────────────────
	 *  Heartbeat counters & TX status
	 * ──────────────────────────────────────────────────────────── */
	uint32_t hb_ya_sent_count; /* how many YA frames we have transmitted     */
	uint32_t hb_hi_recv_count; /* how many HI replies we have received       */
	char hb_tx_status[CHARMANDER_LABEL_LEN]; /* "SENT" | "IDLE"  — flips each send  */
	char hb_rx_status[CHARMANDER_LABEL_LEN]; /* "GOT_HI" | "NO_REPLY" — current state */

	/* ────────────────────────────────────────────────────────────
	 *  Debug / diagnostics
	 * ──────────────────────────────────────────────────────────── */
	uint16_t last_reg_addr;
	uint16_t last_reg_value;
	uint32_t frame_count; /* total valid frames processed  */
	uint32_t crc_error_count; /* frames dropped due to CRC     */
	uint32_t read_req_count; /* FC03 read requests handled    */

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
 * @brief  Low-level heartbeat transmit. Sends FC06 Write reg 0x00 = 22881 ("YA").
 *         Normally called internally by Charmander_Tick(). Exposed for testing.
 */
void Charmander_SendHeartbeat(void);

/**
 * @brief  Build and transmit an FC03 Read-Holding-Registers reply containing
 *         registers 0x00 and 0x26–0x31.  The application must update the READ-side
 *         fields in charmander (sensor_raw, task_raw, position_raw, etc.) before
 *         calling this function or immediately after each motion cycle.
 *         Called automatically by Charmander_Process() when an FC03 request arrives.
 */
void Charmander_BuildReadReply(uint16_t start_addr, uint16_t reg_count);

/* ─── Global instance (defined in charmander.c) ─────────────────── */
extern Charmander_t charmander;

#endif /* CHARMANDER_H */
