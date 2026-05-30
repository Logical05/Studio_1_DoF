/*
 * control.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_CONTROL_CONTROL_H_
#define INC_CONTROL_CONTROL_H_

void Control_Init(void);
void Control_Reset(void);

void Control_UpdateFastISR(void);
void Control_UpdateSlowISR(void);

void Control_SetReference(float position, float velocity, float acceleration);

float Control_GetReferencePosition(void);
float Control_GetReferenceVelocity(void);
float Control_GetReferenceAcceleration(void);

#endif /* INC_CONTROL_CONTROL_H_ */
