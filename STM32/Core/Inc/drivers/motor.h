/*
 * motor.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_DRIVERS_MOTOR_H_
#define INC_DRIVERS_MOTOR_H_

void Motor_Init(void);

void Motor_SetVoltage(float voltage);

void Motor_Brake(void);

float Motor_GetVoltage(void);

#endif /* INC_DRIVERS_MOTOR_H_ */
