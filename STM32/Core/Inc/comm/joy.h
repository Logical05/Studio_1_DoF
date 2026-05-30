/*
 * joy.h
 *
 *  Created on: May 17, 2026
 *      Author: kitth
 */

#ifndef INC_JOY_H_
#define INC_JOY_H_

/**
 ******************************************************************************
 * @file    joy.h
 * @brief   PS-style Joystick Controller Library
 *          Supports simultaneous button presses via bitmask
 ******************************************************************************
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32g4xx_hal.h"

/* ============================================================
 * Button Bitmask Flags — combinable with | operator
 * ============================================================ */
#define JOY_NONE        0x0000U

/* D-Pad */
#define JOY_UP          0x0001U
#define JOY_DOWN        0x0002U
#define JOY_LEFT        0x0004U
#define JOY_RIGHT       0x0008U

/* Face Buttons */
#define JOY_CIRCLE      0x0010U
#define JOY_TRIANGLE    0x0020U
#define JOY_SQUARE      0x0040U
#define JOY_XMARK       0x0080U

/* Menu */
#define JOY_SELECT      0x0100U
#define JOY_START       0x0200U

/* Shoulder */
#define JOY_L1          0x0400U
#define JOY_L2          0x0800U
#define JOY_R1          0x1000U
#define JOY_R2          0x2000U

typedef uint16_t JoyButton_t;

/* ============================================================
 * Analog Axis Data
 * ============================================================ */
typedef struct {
	int16_t x; /* -127 = full left,  0 = center, +127 = full right */
	int16_t y; /* -128 = full up,    0 = center, +128 = full down  */
} JoyAxis_t;

/* ============================================================
 * Joystick State
 * ============================================================ */
typedef struct {
	/* Bitmask — all buttons combined, check with JOY_IsPressed() */
	volatile JoyButton_t buttons;

	/* Analog axes */
	volatile JoyAxis_t left;
	volatile JoyAxis_t right;

	/* Individual flags — easy to watch in Live Expression */
	volatile uint8_t btn_up;
	volatile uint8_t btn_down;
	volatile uint8_t btn_left;
	volatile uint8_t btn_right;
	volatile uint8_t btn_circle;
	volatile uint8_t btn_triangle;
	volatile uint8_t btn_square;
	volatile uint8_t btn_xmark;
	volatile uint8_t btn_select;
	volatile uint8_t btn_start;
	volatile uint8_t btn_l1;
	volatile uint8_t btn_l2;
	volatile uint8_t btn_r1;
	volatile uint8_t btn_r2;
} JoyState_t;

/* ============================================================
 * Public API — Core
 * ============================================================ */
void JOY_Init(UART_HandleTypeDef *huart);
void JOY_RxCpltCallback(UART_HandleTypeDef *huart);
void JOY_Update(void);

const JoyState_t* JOY_GetState(void);
JoyButton_t JOY_GetButtons(void);
JoyAxis_t JOY_GetLeftAxis(void);
JoyAxis_t JOY_GetRightAxis(void);

/* ============================================================
 * Public API — Buttons
 * ============================================================ */

/** Returns 1 if ALL specified buttons are pressed simultaneously
 *  e.g. JOY_IsPressed(JOY_L1 | JOY_R1) → 1 only if BOTH held */
uint8_t JOY_IsPressed(JoyButton_t btn);

/** Returns 1 if ANY of the specified buttons are pressed */
uint8_t JOY_IsAnyPressed(JoyButton_t btn);

/** Clear a specific button or combination from the bitmask */
void JOY_ClearButton(JoyButton_t btn);

/** Clear all buttons */
void JOY_ClearAllButtons(void);

/* ============================================================
 * Public API — Analog Axes
 * ============================================================ */

/** Left stick X:  -127 (full left)  .. 0 (center) .. +127 (full right) */
int16_t JOY_LeftX(void);

/** Left stick Y:  -128 (full up)    .. 0 (center) .. +128 (full down)  */
int16_t JOY_LeftY(void);

/** Right stick X: -127 (full left)  .. 0 (center) .. +127 (full right) */
int16_t JOY_RightX(void);

/** Right stick Y: -128 (full up)    .. 0 (center) .. +128 (full down)  */
int16_t JOY_RightY(void);

/** Returns 1 if stick magnitude exceeds deadzone threshold, else 0 */
uint8_t JOY_LeftActive(int16_t deadzone);
uint8_t JOY_RightActive(int16_t deadzone);

/** Maps any axis value to -100..+100 percent */
int8_t JOY_AxisPercent(int16_t axis_val);

#ifdef __cplusplus
}
#endif

#endif /* INC_JOY_H_ */
