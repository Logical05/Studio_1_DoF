/*
 * pid.c
 *
 *  Created on: May 3, 2026
 *      Author: rajap
 */

#include "control/pid.h"
#include "shared/robot_math.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_output,
bool anti_windup, float integral_limit) {
	pid->max_output = max_output;
	pid->integral_limit = integral_limit;
	pid->anti_windup = anti_windup;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	PID_Reset(pid);
}

void PID_Reset(PID_TypeDef *pid) {
	pid->error[Error_NOW] = 0.0f;
	pid->error[Error_LAST] = 0.0f;

	pid->p_out = 0.0f;
	pid->i_out = 0.0f;
	pid->d_out = 0.0f;
	pid->output = 0.0f;

	if (pid->anti_windup && pid->ki == 0.0f) pid->anti_windup = false;
}

void PID_Calc(PID_TypeDef *pid, float setpoint, float process) {
	pid->error[Error_NOW] = setpoint - process;

	pid->p_out = pid->kp * pid->error[Error_NOW];
	pid->d_out = pid->kd * (pid->error[Error_NOW] - pid->error[Error_LAST]);

	// Integral Limit
	pid->i_out = clampf_max(pid->i_out + (pid->ki * pid->error[Error_NOW]),
		pid->integral_limit);

	// Anti-Windup Clamp
	bool positive = pid->output >= pid->max_output
			&& pid->output * pid->error[Error_NOW] > 0;
	bool negative = pid->output <= -pid->max_output
			&& pid->output * pid->error[Error_NOW] < 0;
	if (pid->anti_windup && (positive || negative)) {
		pid->i_out = 0.0f;
	}

	pid->output = clampf_max(pid->p_out + pid->i_out + pid->d_out, pid->max_output);

	pid->error[Error_LAST] = pid->error[Error_NOW];
}
