/*
 * mode_home.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "modes/mode_home.h"
#include "config/robot_config.h"

#include "drivers/io.h"
#include "drivers/motor.h"

#include "comm/charmander.h"

void Mode_Home_Update(void) {
    Charmander_SetTask(0x0001); /* 0x27 bit 0 = Homing */

    /*
     * Home sensor triggered
     */
    if (IO_IsHomeSensorTriggered()) {
        Motor_Brake();

        charmander.mode = CHARMANDER_MODE_SET_HOME;

        return;
    }

    /*
     * Move slowly toward home sensor
     */
    Motor_SetVoltage(HOMING_VOLTAGE);
}
