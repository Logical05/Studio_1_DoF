/*
 * robot_math.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_SHARED_ROBOT_MATH_H_
#define INC_SHARED_ROBOT_MATH_H_

#include <stdbool.h>

/*
 * ============================================================
 * CONSTANTS
 * ============================================================
 */

#define PI_F            3.14159265359f
#define TWO_PI_F        6.28318530718f

/*
 * ============================================================
 * ANGLE CONVERSION
 * ============================================================
 */

#define DEG_TO_RAD(x)   ((x) * 0.01745329251f)
#define RAD_TO_DEG(x)   ((x) * 57.2957795131f)
#define INDEX_TO_RAD(x) ((x) * 0.0872664626f) // 5 degrees in radians

/*
 * ============================================================
 * BASIC MATH
 * ============================================================
 */

#define MAXF(a, b) \
    (((a) > (b)) ? (a) : (b))

#define MINF(a, b) \
    (((a) < (b)) ? (a) : (b))

#define ABSF(x) \
    (((x) < 0.0f) ? -(x) : (x))

/*
 * ============================================================
 * CLAMP
 * ============================================================
 */

static inline float clampf(float value, float min, float max) {
	if (value > max) {
		return max;
	}

	if (value < min) {
		return min;
	}

	return value;
}

static inline float clampf_max(float value, float max) {
	return clampf(value, -max, max);
}

/*
 * ============================================================
 * MAP
 * ============================================================
 */

static inline float mapf(float x, float in_min, float in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * ============================================================
 * SIGN
 * ============================================================
 */

static inline float signf_fast(float x) {
	if (x > 0.0f) {
		return 1.0f;
	}

	if (x < 0.0f) {
		return -1.0f;
	}

	return 0.0f;
}

/*
 * ============================================================
 * RANGE
 * ============================================================
 */

static inline bool inrangef(float value, float range) {
	return (-range <= value) && (value <= range);
}

/*
 * ============================================================
 * WRAP ANGLE
 * ============================================================
 */

static inline float wrap_2pi(float angle) {
	while (angle >= TWO_PI_F) {
		angle -= TWO_PI_F;
	}

	while (angle < 0.0f) {
		angle += TWO_PI_F;
	}

	return angle;
}

/*
 * ============================================================
 * FLOAT ABS
 * ============================================================
 */

static inline float fabsf_fast(float x) {
	return ABSF(x);
}

#endif /* INC_SHARED_ROBOT_MATH_H_ */
