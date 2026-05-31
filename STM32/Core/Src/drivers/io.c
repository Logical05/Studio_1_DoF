/*
 * io.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "drivers/io.h"

#include "config/pinmap.h"
#include "gpio.h"

bool IO_IsHomeSensorTriggered(void) {
    return HAL_GPIO_ReadPin(PIN_PROX_IN_PORT, PIN_PROX_IN_PIN) == GPIO_PIN_RESET;
}

bool IO_IsEmergencyPressed(void) {
    return HAL_GPIO_ReadPin(PIN_RELAY_IN_PORT, PIN_RELAY_IN_PIN) == GPIO_PIN_SET;
}

bool IO_IsResetPressed(void) {
    return HAL_GPIO_ReadPin(PIN_RESET_5W_IN_PORT, PIN_RESET_5W_IN_PIN) ==
           GPIO_PIN_RESET;
}

void IO_SetSSR(bool state) {
    HAL_GPIO_WritePin(PIN_SSR_TRIG_PORT, PIN_SSR_TRIG_PIN,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
