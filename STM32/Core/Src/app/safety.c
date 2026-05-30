/*
 * safety.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "app/safety.h"

#include "comm/charmander.h"
#include "config/pinmap.h"

#include "gpio.h"

static bool emergency_button = false;
static bool emergency_latched = true;
static bool emergency_state = true;

void Safety_Init(void) {
	emergency_button = false;
	emergency_latched = true;
	emergency_state = true;
}

void Safety_Update(void) {
	/*
	 * Physical emergency button
	 */
	emergency_button = (HAL_GPIO_ReadPin(PIN_RELAY_IN_PORT, PIN_RELAY_IN_PIN)
			== GPIO_PIN_SET);

	/*
	 * Latch emergency
	 */
	if (emergency_button) {
		emergency_latched = true;
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
	}

	Charmander_SetEmergency(emergency_state ? 1 : 0);
}

bool Safety_IsEmergency(void) {
	return emergency_state;
}

void Safety_ClearLatch(void) {
	emergency_latched = false;
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

	GPIO_PinState state = HAL_GPIO_ReadPin(PIN_RESET_5W_IN_PORT,
	PIN_RESET_5W_IN_PIN);

	if (state == GPIO_PIN_RESET) {
		Safety_ClearLatch();
	}
}
