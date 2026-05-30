/*
 * charmander_bridge.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "comm/charmander_bridge.h"

#include "app/safety.h"
#include "comm/charmander.h"
#include "drivers/gripper.h"
#include "drivers/qei.h"
#include "shared/robot_math.h"
#include "stm32g4xx_hal.h"
#include "usart.h"

static uint8_t rx_byte;

void CharmanderBridge_Init(void) {
    Charmander_Init();

    HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
}

void CharmanderBridge_Update(void) {
    /*
     * Modbus processing
     */
    Charmander_Process();

    /*
     * Heartbeat watchdog
     */
    Charmander_Tick();

    /*
     * Vertical axis
     */
    if (charmander.gripper_vertical == CHARMANDER_VERTICAL_DOWN) {
        Gripper_Command(GRIPPER_CMD_DOWN);

        charmander.gripper_vertical = CHARMANDER_VERTICAL_IDLE;
    } else if (charmander.gripper_vertical == CHARMANDER_VERTICAL_UP) {
        Gripper_Command(GRIPPER_CMD_UP);

        charmander.gripper_vertical = CHARMANDER_VERTICAL_IDLE;
    }

    /*
     * Jaw
     */
    if (charmander.gripper_jaw == CHARMANDER_JAW_CLOSE) {
        Gripper_Command(GRIPPER_CMD_CLOSE);

        charmander.gripper_jaw = CHARMANDER_JAW_IDLE;
    } else if (charmander.gripper_jaw == CHARMANDER_JAW_OPEN) {
        Gripper_Command(GRIPPER_CMD_OPEN);

        charmander.gripper_jaw = CHARMANDER_JAW_IDLE;
    }

    /*
     * Automatic sequence
     */
    if (charmander.gripper_seq == CHARMANDER_SEQ_PICK) {
        Gripper_Command(GRIPPER_CMD_PICK);

        charmander.gripper_seq = CHARMANDER_SEQ_NONE;
    } else if (charmander.gripper_seq == CHARMANDER_SEQ_PLACE) {
        Gripper_Command(GRIPPER_CMD_PLACE);

        charmander.gripper_seq = CHARMANDER_SEQ_NONE;
    }
}

void CharmanderBridge_UpdateFeedback(void) {
    /*
     * Motion feedback
     */
    float position_deg      = RAD_TO_DEG(QEI_GetTheta());
    float velocity_dps      = RAD_TO_DEG(QEI_GetFilteredOmega());
    float acceleration_dps2 = 0.0f;

    Charmander_SetMotion(position_deg, velocity_dps, acceleration_dps2);

    /*
     * Emergency state
     */
    Charmander_SetEmergency(Safety_IsEmergency() ? 1 : 0);

    /*
     * Gripper sensor state
     */
    uint16_t sensors = 0;
    if (Gripper_IsUp()) sensors |= (1 << 0);
    if (Gripper_IsDown()) sensors |= (1 << 1);
    if (Gripper_IsClosed()) sensors |= (1 << 2);
    Charmander_SetSensors(sensors);
}

/*
 * UART receive callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == LPUART1) {
        Charmander_Feed(rx_byte);

        HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
    }
}
