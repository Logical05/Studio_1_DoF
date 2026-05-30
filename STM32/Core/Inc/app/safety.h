/*
 * safety.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_APP_SAFETY_H_
#define INC_APP_SAFETY_H_

#include "stdbool.h"
#include "stdint.h"

void Safety_Init(void);

void Safety_Update(void);

bool Safety_IsEmergency(void);

void Safety_ClearLatch(void);

void Safety_EXTI_Callback(uint16_t pin);

#endif /* INC_APP_SAFETY_H_ */
