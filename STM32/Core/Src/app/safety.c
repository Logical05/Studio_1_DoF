/*
 * safety.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "app/safety.h"

#include "comm/charmander.h"
#include "config/pinmap.h"
#include "drivers/io.h"
#include "gpio.h"

static bool emergency_button = false;
static bool emergency_latched = true;
static bool emergency_state = true;

void Safety_Init(void) {
	emergency_button = false;
	emergency_state = true;
	Safety_SetLatch();
}

void Safety_Update(void) {
	/*
	 * Physical emergency button
	 */
	emergency_button = IO_IsEmergencyPressed();

	/*
	 * Latch emergency
	 */
	if (emergency_button) {
		Safety_SetLatch();
	}

	/*
	 * Communication watchdog
	 */
	bool link_dead = (charmander.heartbeat != CHARMANDER_HB_ALIVE);

	/*
	 * Combined emergency state
	 */
	emergency_state = emergency_button || emergency_latched || link_dead;

	/*
	 * Soft stop from Modbus
	 */
	if (charmander.soft_stop == CHARMANDER_STOP_ACTIVE) {
		emergency_state = true;
		Safety_SetLatch();
	}

	Charmander_SetEmergency(emergency_state ? 1 : 0);
}

bool Safety_IsEmergency(void) {
	return emergency_state;
}

void Safety_ClearLatch(void) {
	IO_SetSSR(false);
	emergency_latched = false;
}

void Safety_SetLatch(void) {
	IO_SetSSR(true);
	emergency_latched = true;
}

void Safety_EXTI_Callback(uint16_t pin) {
	if (pin != PIN_RESET_5W_IN_PIN) {
		return;
	}

	/*
	 * Ignore reset while E-stop pressed
	 */
	if (emergency_button) {
		return;
	}

	if (IO_IsResetPressed()) {
		Safety_ClearLatch();

	}
}
