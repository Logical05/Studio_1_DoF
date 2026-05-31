/*
 * control.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "control/control.h"

#include "config/robot_config.h"
#include "control/kalman.h"
#include "control/pid.h"
#include "drivers/motor.h"
#include "drivers/qei.h"

#include <math.h>

typedef struct {
        float position;
        float velocity;
        float acceleration;
} MotionReference_t;

static MotionReference_t reference = {0};

static PID_TypeDef position_pid = {0};
static PID_TypeDef velocity_pid = {0};

static float Feedforward_Calc(float dq_ref, float ddq_ref);

void Control_Init(void) {
    /*
     * Position loop
     */
    PID_Init(&position_pid, POSITION_KP, POSITION_KI, POSITION_KD, POSITION_LIMIT,
             false, 0.0f);

    /*
     * Velocity loop
     */
    PID_Init(&velocity_pid, VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, VELOCITY_LIMIT,
             true, VELOCITY_INTEGRAL_LIMIT);

    /*
     * Kalman observer
     */
    KF_DCMotor_Init(CONTROL_FAST_DT, MOTOR_J, MOTOR_B, MOTOR_KT, MOTOR_KE, MOTOR_R,
                    MOTOR_L, KF_Q_T, KF_R);
}

void Control_Reset(void) {
    PID_Reset(&position_pid);
    PID_Reset(&velocity_pid);
    Control_SetReference(0.0f, 0.0f, 0.0f);
}

void Control_SetReference(float position, float velocity, float acceleration) {
    reference.position     = position;
    reference.velocity     = velocity;
    reference.acceleration = acceleration;
}

float Control_GetReferencePosition(void) { return reference.position; }

float Control_GetReferenceVelocity(void) { return reference.velocity; }

float Control_GetReferenceAcceleration(void) { return reference.acceleration; }

void Control_UpdateFastISR(void) {
    float omega = QEI_GetFilteredOmega();

    /*
     * Velocity loop
     */
    PID_Calc(&velocity_pid, position_pid.output + reference.velocity, omega);

    /*
     * Feedforward
     */
    float v_ff = Feedforward_Calc(reference.velocity, reference.acceleration);

    /*
     * Disturbance observer
     */
    float tau_disturb = KF_GetLoadTorque();
    float v_disturb   = -(MOTOR_R / MOTOR_KT) * tau_disturb;

    /*
     * Motor output
     */
    float voltage = velocity_pid.output + v_ff + v_disturb;
    Motor_SetVoltage(voltage);
}

void Control_UpdateSlowISR(void) {
    PID_Calc(&position_pid, reference.position, QEI_GetTheta());
}

static float Feedforward_Calc(float dq_ref, float ddq_ref) {
    /*
     * Inverse dynamics
     *
     * tau = J*ddq + B*dq
     */
    float tau_ff = MOTOR_J * ddq_ref + MOTOR_B * dq_ref;

    /*
     * Torque -> voltage
     */
    float v_ff = (MOTOR_R / MOTOR_KT) * tau_ff + MOTOR_KE * dq_ref;

    return v_ff;
}
