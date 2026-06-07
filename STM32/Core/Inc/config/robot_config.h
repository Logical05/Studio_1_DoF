/*
 * robot_config.h
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#ifndef INC_CONFIG_ROBOT_CONFIG_H_
#define INC_CONFIG_ROBOT_CONFIG_H_

/*
 * ============================================================
 * CONTROL TIMING
 * ============================================================
 */

#define CONTROL_FAST_DT 0.0002f
#define CONTROL_SLOW_DT 0.001f

/*
 * ============================================================
 * MOTOR
 * ============================================================
 */

#define MOTOR_MAX_VOLTAGE 24.0f

#define MOTOR_J 0.092718896322627f
#define MOTOR_B 1.036743055651295f

#define MOTOR_KT 2.594477849336442f
#define MOTOR_KE 2.061405129659911f

#define MOTOR_R 9.0473919f
#define MOTOR_L 0.0017419f

/*
* T_fc = MOTOR_KT * VOLT_FF / MOTOR_R
* VOLT_FF is voltage at which motor can just overcome Coulomb friction and start moving.
*/
#define MOTOR_FC 0.8602958326597222f

/*
 * ============================================================
 * ENCODER
 * ============================================================
 */

#define ONE_TURN_QEI_CNT   20480
#define MULTI_TURN_QEI_CNT 61440

/*
 * ============================================================
 * TRAJECTORY
 * ============================================================
 */
#define TRAJ_PICK_VMAX 4.0f
#define TRAJ_PICK_AMAX 2.0f
#define TRAJ_PICK_JMAX 12.0f

#define TRAJ_PLACE_VMAX 2.0f
#define TRAJ_PLACE_AMAX 1.0f
#define TRAJ_PLACE_JMAX 6.0f
/*
 * ============================================================
 * HOMING
 * ============================================================
 */

#define HOMING_VOLTAGE (-3.0f)

/*
 * ============================================================
 * POSITION PID
 * ============================================================
 */

#define POSITION_KP 4.15f
#define POSITION_KI 0.0f
#define POSITION_KD 1e-10f

// #define POSITION_KP 3.0f
// #define POSITION_KI 0.0f
// #define POSITION_KD 0.12f

#define POSITION_LIMIT 5.31f

/*
 * ============================================================
 * VELOCITY PID
 * ============================================================
 */

#define VELOCITY_KP 85.0f
#define VELOCITY_KI 0.06f
#define VELOCITY_KD 0.0f

// #define VELOCITY_KP 6.7f
// #define VELOCITY_KI 0.093f
// #define VELOCITY_KD 0.0f

#define VELOCITY_LIMIT          24.0f
#define VELOCITY_INTEGRAL_LIMIT 10.0f

/*
 * ============================================================
 * KALMAN FILTER
 * ============================================================
 */

#define KF_Q 1.0e-3f
#define KF_R 7.85e-9f

/*
 * ============================================================
 * TEST
 * ============================================================
 */

#define TEST_SWEEP_ANGLE_DEG 355.0f

#endif /* INC_CONFIG_ROBOT_CONFIG_H_ */
