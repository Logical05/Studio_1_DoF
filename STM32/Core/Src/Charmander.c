/**
 * @file    charmander.c
 * @brief   Charmander — Modbus RTU Command Bridge (Slave Implementation)
 *
 * PROTOCOL RULES (STM32 = Slave #21):
 *  - PC (Master) initiates ALL communication.
 *  - Robot (Slave) ONLY replies to FC03 Read polls or acknowledges FC06 Writes.
 *  - Heartbeat is PASSIVE: PC polls reg 0x00 → robot replies 22881 → PC writes 18537 → robot marks ALIVE.
 *  - DO NOT call Charmander_SendHeartbeat() in production. It breaks Modbus timing.
 */

#include "charmander.h"
#include <string.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

/* ─── External UART handle (defined in main.c) ────────────────── */
extern UART_HandleTypeDef hlpuart1;

/* ─── Global instance ───────────────────────────────────────────── */
Charmander_t charmander;

/* ─── Internal ring buffer ──────────────────────────────────────── */
static uint8_t rx_buf[CHARMANDER_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

/* ─── Frame assembly state machine ─────────────────────────────── */
static uint8_t frame[CHARMANDER_WRITE_FRAME_SIZE]; /* 8 bytes covers both FC06 & FC03 requests */
static uint8_t frame_idx = 0;

/* ─── Forward declarations ──────────────────────────────────────── */
static uint16_t crc16(const uint8_t *buf, uint16_t len);
static void decode_write_frame(const uint8_t *f);
static void decode_read_request(const uint8_t *f);
static void update_sensor_labels(void);
static void update_task_label(void);
static void copy_str(char *dst, const char *src);

/* ══════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════ */

void Charmander_Init(void) {
	memset(&charmander, 0, sizeof(charmander));

	/* WRITE-side defaults */
	copy_str(charmander.heartbeat, "WAITING");
	copy_str(charmander.mode, "IDLE");
	copy_str(charmander.gripper_vertical, "IDLE");
	copy_str(charmander.gripper_jaw, "IDLE");
	copy_str(charmander.gripper_jaw_last, "NONE");
	copy_str(charmander.gripper_seq, "NONE");
	copy_str(charmander.gripper_auto, "DISABLED");
	copy_str(charmander.jog_dir, "NONE");
	copy_str(charmander.test_type, "PRECISION");
	copy_str(charmander.prec_unit, "DEGREE");
	copy_str(charmander.p2p_unit, "DEGREE");
	copy_str(charmander.soft_stop, "RUNNING");

	/* READ-side defaults */
	copy_str(charmander.sensor_vertical, "IDLE");
	copy_str(charmander.sensor_jaw, "OPEN");
	copy_str(charmander.task, "IDLE");
	copy_str(charmander.emergency, "NORMAL");

	charmander.hb_last_sent_ms = 0;
	charmander.hb_last_alive_ms = 0;
	charmander.hb_ya_sent_count = 0;
	charmander.hb_hi_recv_count = 0;
	copy_str(charmander.hb_tx_status, "IDLE");
	copy_str(charmander.hb_rx_status, "NO_REPLY");

	frame_idx = 0;
	rx_head = rx_tail = 0;
}

/* ─── Called from ISR — keep fast ───────────────────────────────── */
void Charmander_Feed(uint8_t byte) {
	rx_buf[rx_head & (CHARMANDER_RX_BUF_SIZE - 1)] = byte;
	rx_head++;
}

/* ─── Called from main loop ─────────────────────────────────────── */
void Charmander_Process(void) {
	while (rx_tail != rx_head) {
		uint8_t b = rx_buf[rx_tail & (CHARMANDER_RX_BUF_SIZE - 1)];
		rx_tail++;

		frame[frame_idx++] = b;

		/* ── Byte 0: must be our slave address ── */
		if (frame_idx == 1 && frame[0] != CHARMANDER_SLAVE_ADDR) {
			frame_idx = 0;
			continue;
		}

		/* ── Byte 1: must be FC03 or FC06 ── */
		if (frame_idx == 2&&
		frame[1] != CHARMANDER_FC_WRITE &&
		frame[1] != CHARMANDER_FC_READ) {
			frame_idx = 0;
			continue;
		}

		/* ── Both FC03 and FC06 requests are 8 bytes ── */
		if (frame_idx == CHARMANDER_WRITE_FRAME_SIZE) {
			frame_idx = 0;

			uint16_t crc_calc = crc16(frame, 6);
			uint16_t crc_recv = (uint16_t) frame[7] << 8 | frame[6];

			if (crc_calc != crc_recv) {
				charmander.crc_error_count++;
				continue;
			}

			charmander.frame_count++;

			if (frame[1] == CHARMANDER_FC_WRITE) {
				decode_write_frame(frame);
			} else {
				decode_read_request(frame);
			}
		}
	}
}

/* ═══════════════════════════════════════════════════════════════════
 *  TICK — call every main loop iteration
 *  ONLY monitors heartbeat timeout. Does NOT transmit.
 * ═══════════════════════════════════════════════════════════════════ */
void Charmander_Tick(void) {
	uint32_t now = HAL_GetTick();

	if (charmander.hb_last_alive_ms == 0) {
		copy_str(charmander.heartbeat, "WAITING");
		copy_str(charmander.hb_rx_status, "NO_REPLY");
	} else if ((now - charmander.hb_last_alive_ms) >= CHARMANDER_HB_TIMEOUT_MS) {
		copy_str(charmander.heartbeat, "DEAD");
		copy_str(charmander.hb_rx_status, "NO_REPLY");
	}
}

/* ═══════════════════════════════════════════════════════════════════
 *  ⚠️ TEST-ONLY HEARTBEAT TRANSMIT
 *  Modbus slaves MUST NOT initiate communication.
 *  Call this ONLY for manual debugging with a serial monitor.
 *  NEVER call this from Tick() or your main loop.
 * ═══════════════════════════════════════════════════════════════════ */
void Charmander_SendHeartbeat(void) {
	uint8_t frame_out[8];
	uint16_t crc;

	frame_out[0] = CHARMANDER_SLAVE_ADDR;
	frame_out[1] = CHARMANDER_FC_WRITE;
	frame_out[2] = 0x00;
	frame_out[3] = 0x00;
	frame_out[4] = (uint8_t) (CHARMANDER_HB_YA >> 8);
	frame_out[5] = (uint8_t) (CHARMANDER_HB_YA & 0xFF);

	crc = crc16(frame_out, 6);
	frame_out[6] = (uint8_t) (crc & 0xFF);
	frame_out[7] = (uint8_t) (crc >> 8);

	HAL_UART_Transmit(&hlpuart1, frame_out, 8, 100);

	charmander.hb_ya_sent_count++;
	copy_str(charmander.hb_tx_status, "SENT");
}

/* ═══════════════════════════════════════════════════════════════════
 *  FC03 READ REPLY
 *  Builds and sends the 50-register block (0x00 + 0x26–0x31)
 * ═══════════════════════════════════════════════════════════════════ */
/* ═══════════════════════════════════════════════════════════════════
 *  FC03 READ REPLY
 *  Builds and sends the 50-register block (0x00 + 0x26–0x31)
 * ═══════════════════════════════════════════════════════════════════ */
void Charmander_BuildReadReply(uint16_t start_addr, uint16_t reg_count) {
	/* 1. VISUAL DEBUG: Toggle Green LED every time we reply */
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	/* Validate request bounds */
	if (start_addr != CHARMANDER_READ_START_ADDR)
		return;
	if (reg_count > CHARMANDER_READ_COUNT)
		reg_count = CHARMANDER_READ_COUNT;

	/* Buffer size: Header(3) + Data(2 bytes per reg) + CRC(2) */
	uint8_t reply[3 + CHARMANDER_READ_COUNT * 2 + 2];
	uint16_t byte_count = reg_count * 2;

	/* Header */
	reply[0] = CHARMANDER_SLAVE_ADDR; /* 0x15 */
	reply[1] = CHARMANDER_FC_READ; /* 0x03 */
	reply[2] = (uint8_t) byte_count; /* Byte count */

	/* Prepare Register Data */
	uint16_t regs[CHARMANDER_READ_COUNT];
	memset(regs, 0, sizeof(regs));

	/* 0x00: Heartbeat — CRITICAL: Must be 22881 ("YA") */
	regs[0x00] = CHARMANDER_HB_YA;

	/* 0x26–0x31: Status registers */
	regs[0x26] = charmander.sensor_raw;
	regs[0x27] = charmander.task_raw;
	regs[0x28] = (uint16_t) charmander.position_raw;
	regs[0x29] = (uint16_t) charmander.velocity_raw;
	regs[0x30] = (uint16_t) charmander.accel_raw;
	regs[0x31] = charmander.emergency_raw;

	/* Serialise Registers into Reply Buffer */
	for (uint16_t i = 0; i < reg_count; i++) {
		reply[3 + i * 2] = (uint8_t) (regs[start_addr + i] >> 8); /* Hi Byte */
		reply[3 + i * 2 + 1] = (uint8_t) (regs[start_addr + i] & 0xFF); /* Lo Byte */
	}

	/* Calculate and Append CRC */
	uint16_t total_data_len = 3 + byte_count;
	uint16_t crc = crc16(reply, total_data_len);
	reply[total_data_len] = (uint8_t) (crc & 0xFF); /* CRC Lo */
	reply[total_data_len + 1] = (uint8_t) (crc >> 8); /* CRC Hi */

	/* Transmit the full frame */
	HAL_UART_Transmit(&hlpuart1, reply, total_data_len + 2, 200);

	charmander.read_req_count++;
}

/* ═══════════════════════════════════════════════════════════════════
 *  WRITE FRAME DECODER (FC06)
 * ══════════════════════════════════════════════════════════════════ */
static void decode_write_frame(const uint8_t *f) {
	uint16_t reg = (uint16_t) f[2] << 8 | f[3];
	uint16_t raw = (uint16_t) f[4] << 8 | f[5];
	int16_t s_raw = (int16_t) raw;

	charmander.last_reg_addr = reg;
	charmander.last_reg_value = raw;

	switch (reg) {
	case 0x0000: /* Heartbeat ACK from PC */
		if (raw == CHARMANDER_HB_HI) {
			copy_str(charmander.heartbeat, "ALIVE");
			copy_str(charmander.hb_rx_status, "GOT_HI");
			charmander.hb_last_alive_ms = HAL_GetTick();
			charmander.hb_hi_recv_count++;
		} else {
			copy_str(charmander.heartbeat, "DEAD");
			copy_str(charmander.hb_rx_status, "BAD_VALUE");
		}
		break;

	case 0x0001:
		charmander.mode_raw = raw;
		if (raw & 0x0001)
			copy_str(charmander.mode, "HOME");
		else if (raw & 0x0002)
			copy_str(charmander.mode, "JOG");
		else if (raw & 0x0004)
			copy_str(charmander.mode, "AUTO");
		else if (raw & 0x0008)
			copy_str(charmander.mode, "SET_HOME");
		else if (raw & 0x0010)
			copy_str(charmander.mode, "TEST");
		else
			copy_str(charmander.mode, "IDLE");
		break;

	case 0x0002:
		charmander.gripper_cmd_raw = raw;
		if (raw & 0x0001)
			copy_str(charmander.gripper_vertical, "DOWN");
		else if (raw == 0x0000)
			copy_str(charmander.gripper_vertical, "UP");
		else
			copy_str(charmander.gripper_vertical, "IDLE");

		if (raw & 0x0002) {
			copy_str(charmander.gripper_jaw, "OPEN");
			copy_str(charmander.gripper_jaw_last, "OPEN");
		} else if (raw & 0x0004) {
			copy_str(charmander.gripper_jaw, "CLOSE");
			copy_str(charmander.gripper_jaw_last, "CLOSE");
		} else {
			copy_str(charmander.gripper_jaw, "IDLE");
		}
		break;

	case 0x0003:
		charmander.gripper_seq_raw = raw;
		if (raw & 0x0001)
			copy_str(charmander.gripper_seq, "PICK");
		else if (raw & 0x0002)
			copy_str(charmander.gripper_seq, "PLACE");
		else
			copy_str(charmander.gripper_seq, "NONE");
		break;

	case 0x0004:
		charmander.gripper_auto_raw = raw;
		if (raw & 0x0001)
			copy_str(charmander.gripper_auto, "ENABLED");
		else
			copy_str(charmander.gripper_auto, "DISABLED");
		break;

	case 0x0005:
		charmander.jog_degrees = s_raw;
		if (s_raw > 0)
			copy_str(charmander.jog_dir, "CCW");
		else if (s_raw < 0)
			copy_str(charmander.jog_dir, "CW");
		else
			copy_str(charmander.jog_dir, "NONE");
		break;

	case 0x0006:
		if (raw & 0x0001)
			copy_str(charmander.test_type, "PERFORMANCE");
		else
			copy_str(charmander.test_type, "PRECISION");
		break;

	case 0x0007:
		charmander.perf_velocity = s_raw;
		break;
	case 0x0008:
		charmander.perf_acceleration = s_raw;
		break;
	case 0x0009:
		charmander.prec_init_pos = s_raw;
		break;
	case 0x0010:
		charmander.prec_final_pos = s_raw;
		break;
	case 0x0011:
		charmander.prec_repeat_count = s_raw;
		if (s_raw < 0)
			copy_str(charmander.prec_unit, "INDEX");
		else
			copy_str(charmander.prec_unit, "DEGREE");
		break;

	case 0x0012:
	case 0x0013:
	case 0x0014:
	case 0x0015:
	case 0x0016:
	case 0x0017:
	case 0x0018:
	case 0x0019:
	case 0x001A:
	case 0x001B:
	case 0x001C:
	case 0x001D:
	case 0x001E:
	case 0x001F:
	case 0x0020:
	case 0x0021:
		charmander.pp_slots[reg - 0x0012] = s_raw;
		break;

	case 0x0022:
		charmander.pp_pair_count = raw;
		break;

	case 0x0023:
		if (raw & 0x0001)
			copy_str(charmander.p2p_unit, "INDEX");
		else
			copy_str(charmander.p2p_unit, "DEGREE");
		break;

	case 0x0024:
		charmander.p2p_target = s_raw;
		break;

	case 0x0025:
		if (raw & 0x0001)
			copy_str(charmander.soft_stop, "STOP");
		else
			copy_str(charmander.soft_stop, "RUNNING");
		break;

	default:
		break;
	}
}

/* ═══════════════════════════════════════════════════════════════════
 *  READ REQUEST DECODER (FC03)
 * ═══════════════════════════════════════════════════════════════════ */
static void decode_read_request(const uint8_t *f) {
	uint16_t start = (uint16_t) f[2] << 8 | f[3];
	uint16_t count = (uint16_t) f[4] << 8 | f[5];
	Charmander_BuildReadReply(start, count);
}

/* ═══════════════════════════════════════════════════════════════════
 *  SENSOR & TASK LABEL HELPERS
 * ═══════════════════════════════════════════════════════════════════ */
static void update_sensor_labels(void) {
	uint16_t s = charmander.sensor_raw;
	uint8_t reed1 = (s >> 0) & 0x01;
	uint8_t reed2 = (s >> 1) & 0x01;
	uint8_t reed3 = (s >> 2) & 0x01;

	if (reed1 && !reed2)
		copy_str(charmander.sensor_vertical, "UP");
	else if (!reed1 && reed2)
		copy_str(charmander.sensor_vertical, "DOWN");
	else
		copy_str(charmander.sensor_vertical, "IDLE");

	if (reed3)
		copy_str(charmander.sensor_jaw, "CLOSED");
	else
		copy_str(charmander.sensor_jaw, "OPEN");
}

static void update_task_label(void) {
	uint16_t t = charmander.task_raw;
	if (t & 0x0001)
		copy_str(charmander.task, "HOMING");
	else if (t & 0x0002)
		copy_str(charmander.task, "GO_PICK");
	else if (t & 0x0004)
		copy_str(charmander.task, "GO_PLACE");
	else if (t & 0x0008)
		copy_str(charmander.task, "GO_POINT");
	else
		copy_str(charmander.task, "IDLE");
}

/* ─── Public helpers for application code ───────────────────────── */
void Charmander_SetSensors(uint16_t sensor_bits) {
	charmander.sensor_raw = sensor_bits;
	update_sensor_labels();
}

void Charmander_SetTask(uint16_t task_bits) {
	charmander.task_raw = task_bits;
	update_task_label();
}

void Charmander_SetMotion(float pos, float vel, float acc) {
	charmander.position_real = pos;
	charmander.velocity_real = vel;
	charmander.accel_real = acc;

	charmander.position_raw = (int16_t) (pos * 10.0f);
	charmander.velocity_raw = (int16_t) (vel * 10.0f);
	charmander.accel_raw = (int16_t) (acc * 10.0f);
}

void Charmander_SetEmergency(uint8_t active) {
	charmander.emergency_raw = active ? 0x0001 : 0x0000;
	copy_str(charmander.emergency, active ? "ACTIVE" : "NORMAL");
}

/* ═══════════════════════════════════════════════════════════════════
 *  CRC-16/Modbus & String Helpers
 * ═══════════════════════════════════════════════════════════════════ */
static uint16_t crc16(const uint8_t *buf, uint16_t len) {
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < len; i++) {
		crc ^= buf[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x0001)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc >>= 1;
		}
	}
	return crc;
}

static void copy_str(char *dst, const char *src) {
	strncpy(dst, src, CHARMANDER_LABEL_LEN - 1);
	dst[CHARMANDER_LABEL_LEN - 1] = '\0';
}
