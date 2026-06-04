/*
 * joy_cmd_bridge.c
 *
 * Created on: May 30, 2026
 * Author: rajap
 *
 * Translates JoyState inputs (joy.c) into charmander/safety commands.
 *
 * Joystick Angle Mapping (rotated 90° so X-axis is the control axis):
 *
 *   Thumb pointing LEFT  (-90° after rotation) →  3.0V  (minimum)
 *   Thumb pointing UP    (  0° after rotation) →  6.5V  (center/default)
 *   Thumb pointing RIGHT (+90° after rotation) → 10.0V  (maximum)
 *
 *   Physical feel: rotate the stick like a dial — left for less, right for more.
 *   Pointing straight up = center = 6.5V (resting position).
 *
 * Stability features:
 *  - Hysteresis: ignores small tremor movements below threshold
 *  - Slew rate limiter: smooths out voltage changes per tick
 */

#include <control/joy_cmd_bridge.h>
#include "app/safety.h"
#include "comm/charmander.h"
#include "comm/joy.h"
#include "config/robot_config.h"
#include "control/control.h"
#include "drivers/motor.h"
#include "drivers/qei.h"

#include <math.h>
#include <stdlib.h>

/* ============================================================
 * Private defines
 * ============================================================ */

/* Degrees per jog step for L2 / R2 */
#define JOY_CMD_JOG_STEP_DEG 10

/* Slew rate: max voltage change per Update() call (~10ms tick assumed).
 * Tune: increase if feels sluggish, decrease if still jittery. */
#define JOY_CMD_SLEW_RATE 0.15f /* Volts per tick */

/* Hysteresis: ignore knob movement smaller than this (in raw units out of ±127).
 * Tune: increase if jittery, decrease if unresponsive to small moves. */
#define JOY_CMD_HYSTERESIS 8

/* Minimum stick magnitude to bother computing angle (avoids noise at center) */
#define JOY_CMD_MAGNITUDE_MIN 20

/* Voltage limits */
#define JOY_CMD_VOLT_MIN    (-6.5f)
#define JOY_CMD_VOLT_CENTER (0.0f)
#define JOY_CMD_VOLT_MAX    (6.5f)

/* ============================================================
 * Private state
 * ============================================================ */
static JoyButton_t _prev_buttons = JOY_NONE;
static bool _joy_in_use = false;

static float _target_voltage = JOY_CMD_VOLT_CENTER;
static float _output_voltage = JOY_CMD_VOLT_CENTER;
static int16_t _last_raw_x = 0;
static int16_t _last_raw_y = 0;

/* ============================================================
 * Private helpers
 * ============================================================ */

/*
 * Maps the joystick angle (both X and Y axes) to 3.0V - 10.0V.
 *
 * The stick is treated as a dial rotated 90 degrees so that:
 *   - Straight UP  (y = -127, x = 0)  -> center  -> 6.5V
 *   - Rotated RIGHT (x = +127, y = 0) -> full     -> 10.0V
 *   - Rotated LEFT  (x = -127, y = 0) -> minimum  -> 3.0V
 *
 * atan2(x, -y) gives 0 when pointing up, +90 when right, -90 when left.
 * We clamp to +/-90 so the useful range is only the top half of the circle.
 */
static float _KnobToVoltage_Angle(int16_t raw_x, int16_t raw_y) {
	const int32_t mag_sq = (int32_t) raw_x * raw_x + (int32_t) raw_y * raw_y;

	if (mag_sq < (JOY_CMD_MAGNITUDE_MIN * JOY_CMD_MAGNITUDE_MIN)) {
		return 0.0f;
	}

	float angle_deg = atan2f((float) raw_y, fabsf((float) raw_x))
			* (180.0f / (float) M_PI);

	if (angle_deg > 90.0f) {
		angle_deg = 90.0f;
	} else if (angle_deg < -90.0f) {
		angle_deg = -90.0f;
	}

	return angle_deg * (JOY_CMD_VOLT_MAX / 90.0f);
}

/*
 * Slew rate limiter — smooths voltage changes to avoid sudden jumps.
 * Called every tick. Returns the new smoothed output voltage.
 */
static float _SlewVoltage(float target, float current) {
	float diff = target - current;
	if (diff > JOY_CMD_SLEW_RATE) diff = JOY_CMD_SLEW_RATE;
	if (diff < -JOY_CMD_SLEW_RATE) diff = -JOY_CMD_SLEW_RATE;
	return current + diff;
}

/*
 * Returns 1 on the rising edge of a button (just pressed this tick).
 */
static inline uint8_t _Rising(JoyButton_t btn, JoyButton_t cur, JoyButton_t prev) {
	return ((cur & btn) && !(prev & btn)) ? 1u : 0u;
}

/*
 * Returns true if the motor axis is currently claimed by the knob.
 */
bool JoyInUse(void) {
	return _joy_in_use;
}

/* ============================================================
 * Public API
 * ============================================================ */

void JoyCmdBridge_Init(void) {
	_prev_buttons = JOY_NONE;
	_joy_in_use = false;
	_target_voltage = JOY_CMD_VOLT_CENTER;
	_output_voltage = JOY_CMD_VOLT_CENTER;
	_last_raw_x = 0;
	_last_raw_y = 0;
}

void JoyCmdBridge_Update(void) {
	/* ── 0. Snapshot live state ──────────────────────────────────── */
	const JoyState_t *joy = JOY_GetState();
	JoyButton_t cur = joy->buttons;
	JoyButton_t prev = _prev_buttons;

	int16_t knob_x = joy->right.x;
	int16_t knob_y = joy->right.y;

	/* ── 1. Square -> Emergency latch ────────────────────────────── */
	if (_Rising(JOY_SQUARE, cur, prev)) {
		Motor_SetVoltage(0.0f);
		Safety_SetLatch();
		_joy_in_use = false;
		_output_voltage = JOY_CMD_VOLT_CENTER;
		_target_voltage = JOY_CMD_VOLT_CENTER;
		_prev_buttons = cur;
		return;
	}

	//    /* ── 2. X (cross) -> Soft-stop ───────────────────────────────── */
	//    if (_Rising(JOY_XMARK, cur, prev)) {
	//        Motor_SetVoltage(0.0f);
	//        charmander.soft_stop = CHARMANDER_STOP_ACTIVE;
	//        _joy_in_use          = false;
	//        _output_voltage      = JOY_CMD_VOLT_CENTER;
	//        _target_voltage      = JOY_CMD_VOLT_CENTER;
	//        _prev_buttons        = cur;
	//        return;
	//    }

	if (charmander.mode != CHARMANDER_MODE_IDLE) {
		_prev_buttons = cur;
		return;
	}

	/* ── 3. Right knob -> Direct motor voltage ───────────────────── */
	if (!Safety_IsEmergency()) {
		const int32_t mag_sq = (int32_t) knob_x * knob_x + (int32_t) knob_y * knob_y;

		/* Engage / release hysteresis */
		if (!_joy_in_use) {
			if (mag_sq > (JOY_CMD_MAGNITUDE_MIN * JOY_CMD_MAGNITUDE_MIN)) {
				_joy_in_use = true;
			}
		} else {
			if (mag_sq < (JOY_CMD_MAGNITUDE_MIN * JOY_CMD_MAGNITUDE_MIN)) {
				_joy_in_use = false;
			}
		}

		if (_joy_in_use) {

			/* Update target only when stick position changes enough */
			int dx = (int) knob_x - (int) _last_raw_x;
			int dy = (int) knob_y - (int) _last_raw_y;

			const int dist_sq = dx * dx + dy * dy;

			if (dist_sq > (JOY_CMD_HYSTERESIS * JOY_CMD_HYSTERESIS)) {

				_last_raw_x = knob_x;
				_last_raw_y = knob_y;

				_target_voltage = -_KnobToVoltage_Angle(knob_x, knob_y);
			}

			/* Continue slewing every tick */
			_output_voltage = _SlewVoltage(_target_voltage, _output_voltage);

			Motor_SetVoltage(_output_voltage);

			charmander.mode = CHARMANDER_MODE_IDLE;

			Control_SetReference(QEI_GetTheta(), 0.0f, 0.0f);
			Control_Reset();
		} else {

			_target_voltage = JOY_CMD_VOLT_CENTER;

			_output_voltage = _SlewVoltage(_target_voltage, _output_voltage);

//			Motor_SetVoltage(_output_voltage);

			_last_raw_x = knob_x;
			_last_raw_y = knob_y;
		}
	}

	/* ── 4. Start -> Home mode ───────────────────────────────────── */
	if (_Rising(JOY_START, cur, prev) && !JoyInUse()) {
//		Motor_SetVoltage(0.0f);
		charmander.mode = CHARMANDER_MODE_HOME;
	}

	/* ── 5. Select -> Set-Home mode ──────────────────────────────── */
	if (_Rising(JOY_SELECT, cur, prev) && !JoyInUse()) {
//		Motor_SetVoltage(0.0f);
		charmander.mode = CHARMANDER_MODE_SET_HOME;
	}

	/* ── 6. L2 -> Jog CCW 10 deg ─────────────────────────────────── */
	if (_Rising(JOY_L2, cur, prev) && !JoyInUse()) {
//		Motor_SetVoltage(0.0f);
		charmander.mode = CHARMANDER_MODE_JOG;
		charmander.jog_degrees = (int16_t) (JOY_CMD_JOG_STEP_DEG);
		charmander.jog_dir = CHARMANDER_JOG_CCW;
	}

	/* ── 7. R2 -> Jog CW 10 deg ──────────────────────────────────── */
	if (_Rising(JOY_R2, cur, prev) && !JoyInUse()) {
//		Motor_SetVoltage(0.0f);
		charmander.mode = CHARMANDER_MODE_JOG;
		charmander.jog_degrees = (int16_t) (-JOY_CMD_JOG_STEP_DEG);
		charmander.jog_dir = CHARMANDER_JOG_CW;
	}

	/* ── 8. Up button -> Jaw OPEN ────────────────────────────────── */
	if (_Rising(JOY_UP, cur, prev)) {
		charmander.gripper_jaw = CHARMANDER_JAW_OPEN;
	}

	/* ── 9. Down button -> Jaw CLOSE ─────────────────────────────── */
	if (_Rising(JOY_DOWN, cur, prev)) {
		charmander.gripper_jaw = CHARMANDER_JAW_CLOSE;
	}

	/* ── 10. Left button -> Gripper vertical UP ──────────────────── */
	if (_Rising(JOY_LEFT, cur, prev)) {
		charmander.gripper_vertical = CHARMANDER_VERTICAL_UP;
	}

	/* ── 11. Right button -> Gripper vertical DOWN ───────────────── */
	if (_Rising(JOY_RIGHT, cur, prev)) {
		charmander.gripper_vertical = CHARMANDER_VERTICAL_DOWN;
	}

	/* ── 12. L1 -> Gripper PICK sequence ─────────────────────────── */
	if (_Rising(JOY_L1, cur, prev)) {
		charmander.gripper_seq = CHARMANDER_SEQ_PICK;
	}

	/* ── 13. R1 -> Gripper PLACE sequence ────────────────────────── */
	if (_Rising(JOY_R1, cur, prev)) {
		charmander.gripper_seq = CHARMANDER_SEQ_PLACE;
	}

	/* ── Save button state for next edge detection ───────────────── */
	_prev_buttons = cur;
}
