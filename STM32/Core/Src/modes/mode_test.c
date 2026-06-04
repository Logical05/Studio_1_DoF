/*
 * mode_test.c
 *
 *  Created on: May 29, 2026
 *      Author: rajap
 */

#include "modes/mode_test.h"

#include "comm/charmander.h"
#include "config/robot_config.h"
#include "control/control.h"
#include "control/trajectory_manager.h"
#include "drivers/qei.h"
#include "shared/robot_math.h"

#include <stdbool.h>
#include <stdlib.h>

static float test_points[64];

static bool started = false;

void Mode_Test_Precision(void) {
	/*
	 * Build trajectory once
	 */
	if (!started) {
		float start;
		float end;

		/*
		 * +repeat = degree mode
		 * -repeat = index mode
		 */
		if (charmander.prec_repeat_count >= 0) {
			start = DEG_TO_RAD(charmander.prec_init_pos);
			end = DEG_TO_RAD(charmander.prec_final_pos);
		} else {
			start = INDEX_TO_RAD(charmander.prec_init_pos);
			end = INDEX_TO_RAD(charmander.prec_final_pos);
		}

		uint8_t repeat = (uint8_t) abs(charmander.prec_repeat_count);

		/*
		 * Prevent overflow
		 */
		if (repeat > 32U) {
			repeat = 32U;
		}

		for (uint8_t i = 0; i < (repeat * 2U); i += 2U) {
			test_points[i] = end;
			test_points[i + 1U] = start;
		}

		Trajectory_Start(test_points, repeat * 2U, QEI_GetTheta(), TRAJ_PICK_VMAX,
		TRAJ_PICK_AMAX, TRAJ_PICK_JMAX, TRAJECTORY_SCURVE);

		started = true;
	}

	/*
	 * Advance trajectory
	 */
	if (Trajectory_GetState() == TRAJECTORY_WAIT) {
		Trajectory_Continue();
	}

	/*
	 * Motion reference
	 */
	Control_SetReference(Trajectory_GetPosition(), Trajectory_GetVelocity(),
		Trajectory_GetAcceleration());

	/*
	 * Finished
	 */
	if (Trajectory_IsFinished()) {
		started = false;
		charmander.mode = CHARMANDER_MODE_IDLE;
	}
}

void Mode_Test_Performance(void) {
	if (!started) {
		float current_position = QEI_GetTheta();
		float target_position = current_position + DEG_TO_RAD(TEST_SWEEP_ANGLE_DEG);

		int16_t perf_velocity = charmander.perf_velocity;
		int16_t perf_acceleration = charmander.perf_acceleration;

		perf_velocity = clampf_max(perf_velocity, TRAJ_PICK_VMAX);
		perf_acceleration = clampf_max(perf_acceleration, TRAJ_PICK_AMAX);

		Trajectory_Start(&target_position, 1U, current_position, perf_velocity,
			perf_acceleration, TRAJ_PICK_JMAX, TRAJECTORY_SCURVE);

		started = true;
	}

	/*
	 * Advance trajectory
	 */
	if (Trajectory_GetState() == TRAJECTORY_WAIT) {
		Trajectory_Continue();
	}

	/*
	 * Motion reference
	 */
	Control_SetReference(Trajectory_GetPosition(), Trajectory_GetVelocity(),
		Trajectory_GetAcceleration());
	/*
	 * Finished
	 */
	if (Trajectory_IsFinished()) {
		started = false;
		charmander.mode = CHARMANDER_MODE_IDLE;
	}
}

void Mode_Test_Update(void) {
	switch (charmander.test_type) {
		case CHARMANDER_TEST_PRECISION:
			Mode_Test_Precision();
			break;
		case CHARMANDER_TEST_PERFORMANCE:
			Mode_Test_Performance();
			break;
		default:
			charmander.mode = CHARMANDER_MODE_IDLE;
			break;
	}
}
