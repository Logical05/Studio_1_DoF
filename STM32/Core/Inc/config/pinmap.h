/*
 * pinmap.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_CONFIG_PINMAP_H_
#define INC_CONFIG_PINMAP_H_

#include "main.h"

/*
 * ============================================================
 * SSR
 * ============================================================
 */

#define PIN_SSR_TRIG_PORT          GPIOC
#define PIN_SSR_TRIG_PIN           GPIO_PIN_0

/*
 * ============================================================
 * MOTOR DIRECTION
 * ============================================================
 */

#define PIN_MOTOR_DIR_PORT         GPIOB
#define PIN_MOTOR_DIR_PIN          GPIO_PIN_11

/*
 * ============================================================
 * EMERGENCY RELAY INPUT
 * ============================================================
 */

#define PIN_RELAY_IN_PORT          GPIOC
#define PIN_RELAY_IN_PIN           GPIO_PIN_12

/*
 * ============================================================
 * RESET BUTTON
 * ============================================================
 */

#define PIN_RESET_5W_IN_PORT       GPIOC
#define PIN_RESET_5W_IN_PIN        GPIO_PIN_11

/*
 * ============================================================
 * HOME SENSOR
 * ============================================================
 */

#define PIN_PROX_IN_PORT           GPIOC
#define PIN_PROX_IN_PIN            GPIO_PIN_3

/*
 * ============================================================
 * STATUS LED
 * ============================================================
 */

#define PIN_STATUS_LED_PORT        GPIOA
#define PIN_STATUS_LED_PIN         GPIO_PIN_5

/*
 * ============================================================
 * GRIPPER
 * ============================================================
 */

#define GRIPPER_OPEN_OUT_PORT GPIOC
#define GRIPPER_OPEN_OUT_PIN GPIO_PIN_1

#define GRIPPER_CLOSE_OUT_PORT GPIOB
#define GRIPPER_CLOSE_OUT_PIN GPIO_PIN_14

#define GRIPPER_UP_OUT_PORT GPIOB
#define GRIPPER_UP_OUT_PIN GPIO_PIN_7

#define GRIPPER_DOWN_OUT_PORT GPIOA
#define GRIPPER_DOWN_OUT_PIN GPIO_PIN_15

#define GRIPPER_REED_CLOSE_PORT GPIOB
#define GRIPPER_REED_CLOSE_PIN GPIO_PIN_15

#define GRIPPER_REED_UP_PORT GPIOB
#define GRIPPER_REED_UP_PIN GPIO_PIN_2

#define GRIPPER_REED_DOWN_PORT GPIOB
#define GRIPPER_REED_DOWN_PIN GPIO_PIN_1

#endif /* INC_CONFIG_PINMAP_H_ */
