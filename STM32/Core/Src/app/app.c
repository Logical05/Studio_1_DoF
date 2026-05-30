/*
 * app.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "app/app.h"

#include "app/mode_manager.h"
#include "app/safety.h"
#include "comm/charmander_bridge.h"
#include "config/pinmap.h"
#include "control/control.h"
#include "drivers/gripper.h"
#include "drivers/motor.h"
#include "drivers/qei.h"
#include "gpio.h"

void App_Init(void) {
    Safety_Init();

    Motor_Init();
    QEI_Init();
    Gripper_Init();
    Control_Init();

    CharmanderBridge_Init();
    ModeManager_Init();
}

void App_Update(void) {
    /*
     * Synchronize safety state
     */
    Safety_Update();

    /*
     * Push feedback to Modbus
     */
    CharmanderBridge_Update();
    CharmanderBridge_UpdateFeedback();

    /*
     * Update gripper FSM
     */
    Gripper_Update();

    /*
     * Emergency handling
     */
    if (Safety_IsEmergency()) {
        Motor_Brake();
        ModeManager_ForceIdle();
        return;
    }

    /*
     * Main mode execution
     */
    ModeManager_Update();

    /*
     * Heartbeat LED
     */
    HAL_GPIO_TogglePin(PIN_STATUS_LED_PORT, PIN_STATUS_LED_PIN);
}
