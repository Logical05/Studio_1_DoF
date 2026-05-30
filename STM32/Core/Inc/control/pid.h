/*
 * pid.h
 *
 *  Created on: May 3, 2026
 *      Author: rajap
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdbool.h>

typedef enum {
	Error_NOW, Error_LAST
} ErrorState_t;

typedef struct {
	float kp;
	float ki;
	float kd;
	float error[2];

	float p_out;
	float i_out;
	float d_out;
	float output;

	float max_output;
	float integral_limit;
	bool anti_windup;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_output,
bool anti_windup, float integral_limit);

void PID_Reset(PID_TypeDef *pid);

void PID_Calc(PID_TypeDef *pid, float setpoint, float process);

#endif /* INC_PID_H_ */
