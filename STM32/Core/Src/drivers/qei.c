/*
 * qei.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "drivers/qei.h"

#include "config/robot_config.h"
#include "shared/robot_math.h"
#include "tim.h"

QEI_State_t qei = {0};

void QEI_Init(void) {
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    qei.now  = __HAL_TIM_GET_COUNTER(&htim8);
    qei.prev = qei.now;
}

void QEI_UpdateISR(void) {
    qei.now = __HAL_TIM_GET_COUNTER(&htim8);

    int32_t diff = (int32_t)qei.now - (int32_t)qei.prev;

    /*
     * Encoder wrap handling
     */
    if (diff > (MULTI_TURN_QEI_CNT / 2)) {
        diff -= MULTI_TURN_QEI_CNT;
    } else if (diff < -(MULTI_TURN_QEI_CNT / 2)) {
        diff += MULTI_TURN_QEI_CNT;
    }

    /*
     * Position
     */
    float dtheta = (float)diff * (TWO_PI_F / ONE_TURN_QEI_CNT);

    qei.theta += dtheta;

    /*
     * Velocity
     */
    qei.omega = dtheta / CONTROL_FAST_DT;

    /*
     * Store
     */
    qei.prev = qei.now;
}

void QEI_Reset(void) {
    qei.theta          = 0.0f;
    qei.omega          = 0.0f;
    qei.omega_filtered = 0.0f;

    qei.now = __HAL_TIM_GET_COUNTER(&htim8);

    qei.prev = qei.now;
}

float QEI_GetTheta(void) { return qei.theta; }

float QEI_GetOmega(void) { return qei.omega; }

float QEI_GetFilteredOmega(void) { return qei.omega_filtered; }
