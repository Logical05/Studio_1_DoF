/*
 * qei.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_DRIVERS_QEI_H_
#define INC_DRIVERS_QEI_H_

#include <stdint.h>

typedef struct {
        uint32_t now;
        uint32_t prev;

        float theta;
        float omega;
        float omega_filtered;
} QEI_State_t;

extern QEI_State_t qei;

void QEI_Init(void);

void QEI_UpdateISR(void);

void QEI_Reset(void);

float QEI_GetTheta(void);

float QEI_GetOmega(void);

float QEI_GetFilteredOmega(void);

#endif /* INC_DRIVERS_QEI_H_ */
