/*
 * motor.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "drivers/motor.h"

#include "config/pinmap.h"
#include "config/robot_config.h"
#include "gpio.h"
#include "main.h"
#include "shared/robot_math.h"
#include "tim.h"

static float motor_voltage = 0.0f;

void Motor_Init(void) {
    Motor_Brake();

    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void Motor_SetVoltage(float voltage) {
    /*
     * Brake at zero command
     */
    if (fabsf_fast(voltage) < 0.0001f) {
        Motor_Brake();
        return;
    }

    /*
     * Clamp voltage
     */
    voltage = clampf(voltage, -MOTOR_MAX_VOLTAGE, MOTOR_MAX_VOLTAGE);

    motor_voltage = voltage;

    /*
     * Set direction
     */
    HAL_GPIO_WritePin(PIN_MOTOR_DIR_PORT, PIN_MOTOR_DIR_PIN,
                      (voltage > 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /*
     * Convert voltage -> PWM
     */
    float duty = fabsf_fast(voltage) / MOTOR_MAX_VOLTAGE;

    uint16_t pwm = (uint16_t)(duty * 100.0f);

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwm);
    return;
}

void Motor_Brake(void) {
    motor_voltage = 0.0f;

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);

    HAL_GPIO_WritePin(PIN_MOTOR_DIR_PORT, PIN_MOTOR_DIR_PIN, GPIO_PIN_RESET);
}

float Motor_GetVoltage(void) { return motor_voltage; }