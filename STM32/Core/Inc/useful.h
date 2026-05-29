/*
 * useful.h
 *
 *  Created on: May 4, 2026
 *      Author: rajap
 */

#ifndef SRC_USEFUL_H_
#define SRC_USEFUL_H_

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <math.h>

#define M_2PI 6.28318530717958647692

#define RAD_TO_DEG(r) (((float)r) * (180.0f / (float)M_PI))
#define DEG_TO_RAD(r) (((float)r) * ((float)M_PI / 180.0f))

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} Pinout_t;

typedef struct {
	uint32_t start;
	uint32_t duration;
	bool active;
} DelayTimer;

static void Timer_Start(DelayTimer *t, uint32_t ms) {
	t->start = HAL_GetTick();
	t->duration = ms;
	t->active = true;
}

static bool Timer_Expired(DelayTimer *t) {
	if (!t->active) return false;

	if ((HAL_GetTick() - t->start) >= t->duration) {
		t->active = false;
		return true;
	}

	return false;
}

static inline float map(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float clamp(float x, float min, float max) {
	if (x < min) return min;
	if (x > max) return max;
	return x;
}

static inline float clamp_max(float x, float max) {
	return clamp(x, -max, max);
}

static inline float signf_fast(float x) {
	return (x > 0.0f) - (x < 0.0f);
}

#endif /* SRC_USEFUL_H_ */
