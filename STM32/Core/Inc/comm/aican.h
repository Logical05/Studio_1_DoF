/*
 * aican.h
 *
 *  Created on: Jun 3, 2026
 *      Author: kitth
 */

#ifndef INC_COMM_AICAN_H_
#define INC_COMM_AICAN_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* --- CAN Network Configurations --- */
#define AICAN_NODE_ID             0x10
#define AICAN_MASTER_ID           0x00

#define AICAN_FUNC_CMD_REQ        0x2
#define AICAN_FUNC_CMD_RESP       0x3
#define AICAN_FUNC_HEARTBEAT      0x6

#define AICAN_RELAY_BANK          0x00
#define AICAN_OPTO_BANK           0x10

#define AICAN_CMD_WRITE_REQ       0x10
#define AICAN_CMD_WRITE_ACK       0x11
#define AICAN_CMD_READ_REQ        0x20
#define AICAN_CMD_READ_RESP       0x21

/* --- Watchdog Configuration --- */
#define AICAN_WATCHDOG_TIMEOUT_MS  2000

/* --- ID Generation Macro --- */
#define MAKE_CAN_ID(funcCode, nodeID) (((funcCode & 0x07) << 8) | (nodeID & 0xFF))

/* --- Public API Declarations --- */

/**
 * @brief Initializes filters and starts the FDCAN1 hardware instance.
 * @param hfdcan Pointer to the initialized CubeMX FDCAN handle.
 */
void AICAN_Init(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Checks for network timeouts and updates safe states.
 * Must be called periodically inside App_Update().
 */
void AICAN_Update(void);

/**
 * @brief Processes incoming messages from the FDCAN FIFO.
 * Called automatically via the hardware FIFO interrupt callback.
 */
void AICAN_ProcessInterrupt(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
 * @brief Dispatches physical limit switches status back to the CAN Master bus.
 * @param sensor_mask Mask containing live state of reed switches (Bit 0=Up, 1=Down, 2=Closed)
 */
void AICAN_SendReadResponse(uint8_t sensor_mask);

#endif /* INC_COMM_AICAN_H_ */
