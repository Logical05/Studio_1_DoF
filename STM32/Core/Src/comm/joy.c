/*
 * joy.c
 *
 *  Created on: May 17, 2026
 *      Author: kitth
 */

#include "comm/joy.h"

#include <stdlib.h>
#include <string.h>

/* ============================================================
 * Private Defines
 * ============================================================ */
#define JOY_PACKET_BUF_SIZE 32
#define JOY_BTN_TIMEOUT_MS  200

/* ============================================================
 * Private Variables
 * ============================================================ */
static UART_HandleTypeDef *_huart   = NULL;
static uint8_t             _rx_byte = 0;

static JoyState_t _state = {
    .buttons      = JOY_NONE,
    .left         = {.x = 0, .y = 0},
    .right        = {.x = 0, .y = 0},
    .btn_up       = 0,
    .btn_down     = 0,
    .btn_left     = 0,
    .btn_right    = 0,
    .btn_circle   = 0,
    .btn_triangle = 0,
    .btn_square   = 0,
    .btn_xmark    = 0,
    .btn_select   = 0,
    .btn_start    = 0,
    .btn_l1       = 0,
    .btn_l2       = 0,
    .btn_r1       = 0,
    .btn_r2       = 0,
};

static uint8_t _packet_buf[JOY_PACKET_BUF_SIZE];
static uint8_t _packet_idx = 0;

/* Separate tracking ticks to prevent D-pad and Action buttons from refreshing each
 * other */
static uint32_t _last_dpad_tick   = 0;
static uint32_t _last_action_tick = 0;

/* Active bitmasks separated by category */
static JoyButton_t _active_dpad    = JOY_NONE;
static JoyButton_t _active_actions = JOY_NONE;

/* ============================================================
 * Debug — Live Expression monitor
 * Add "joy_debug" to Live Expression — expands to all 16 slots.
 *
 *  Index map:
 *   [0]  UP        [1]  DOWN      [2]  LEFT      [3]  RIGHT
 *   [4]  CIRCLE    [5]  TRIANGLE  [6]  SQUARE    [7]  XMARK
 *   [8]  SELECT    [9]  START     [10] L1        [11] L2
 *   [12] R1        [13] R2        [14] last rx_byte (raw hex)
 *   [15] packet_idx (>0 means byte was swallowed as analog packet)
 * ============================================================ */
volatile uint8_t joy_debug[16] = {0};

/* ============================================================
 * Private Helpers — forward declarations
 * ============================================================ */
static JoyButton_t    _MapByte(uint8_t byte);
static void           _ParsePacket(void);
static void           _SyncFlags(void);
static inline int16_t _Remap(uint8_t raw, uint8_t center);

/* ============================================================
 * Public API — Init & Update
 * ============================================================ */

void JOY_Init(UART_HandleTypeDef *huart) {
    _huart            = huart;
    _packet_idx       = 0;
    _active_dpad      = JOY_NONE;
    _active_actions   = JOY_NONE;
    _last_dpad_tick   = 0;
    _last_action_tick = 0;
    memset(_packet_buf, 0, sizeof(_packet_buf));
    memset(&_state, 0, sizeof(_state));
    memset((void *)joy_debug, 0, sizeof(joy_debug));

    /* Axes start at center (0) */
    _state.left.x  = 0;
    _state.left.y  = 0;
    _state.right.x = 0;
    _state.right.y = 0;

    HAL_UART_Receive_IT(_huart, &_rx_byte, 1);
}

void JOY_Update(void) {
    uint32_t current_tick = HAL_GetTick();
    uint8_t  changed      = 0;

    /* Check if D-Pad has timed out independently */
    if (_active_dpad != JOY_NONE &&
        (current_tick - _last_dpad_tick) >= JOY_BTN_TIMEOUT_MS) {
        _active_dpad = JOY_NONE;
        changed      = 1;
    }

    /* Check if Action buttons have timed out independently */
    if (_active_actions != JOY_NONE &&
        (current_tick - _last_action_tick) >= JOY_BTN_TIMEOUT_MS) {
        _active_actions = JOY_NONE;
        changed         = 1;
    }

    /* If a timeout occurred, merge state and update live expression flags */
    if (changed) {
        _state.buttons = _active_dpad | _active_actions;
        _SyncFlags();
    }
}

/* ============================================================
 * Private Helper: Process incoming button byte cleanly
 * ============================================================ */
static void _ProcessButtonByte(uint8_t b) {
    JoyButton_t btn = _MapByte(b);

    if (btn != JOY_NONE) {
        uint32_t current_tick = HAL_GetTick();

        if (btn == JOY_UP || btn == JOY_DOWN || btn == JOY_LEFT ||
            btn == JOY_RIGHT) {
            /* D-Pad event received. Update dpad mask and refresh ITS OWN timer only
             */
            _active_dpad    = btn;
            _last_dpad_tick = current_tick;
        } else {
            /* Action/Face button event received. Accumulate and refresh ITS OWN
             * timer */
            _active_actions   |= btn;
            _last_action_tick  = current_tick;
        }

        /* Combine both active categories to generate current global state */
        _state.buttons = _active_dpad | _active_actions;
        _SyncFlags();
    }
}

/* ============================================================
 * Public API — ISR Callback (call from HAL_UART_RxCpltCallback)
 * ============================================================ */

void JOY_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (_huart == NULL || huart->Instance != _huart->Instance) return;

    uint8_t b = _rx_byte;

    /* Always mirror the raw byte for Live Expression debugging */
    joy_debug[14] = b;
    joy_debug[15] = _packet_idx;

    if (b == 0x0A) {
        /* LF — end of ASCII analog packet, parse it */
        _packet_buf[_packet_idx] = '\0';
        _packet_idx              = 0;
        _ParsePacket();

    } else if (b == 0x0D) {
        /* CR — skip */

    } else if (b == 'W' || b == 'Q') {
        /* Start of a new analog packet */
        _packet_idx                = 0;
        _packet_buf[_packet_idx++] = b;

    } else if (_packet_idx > 0) {
        /* Mid-packet: accumulate digits, 'P', 'S' */
        if ((b >= '0' && b <= '9') || b == 'P' || b == 'S') {
            if (_packet_idx < JOY_PACKET_BUF_SIZE - 1)
                _packet_buf[_packet_idx++] = b;
        } else {
            /* Unexpected byte mid-packet — abort packet, treat as button */
            _packet_idx = 0;
            _ProcessButtonByte(b);
        }

    } else {
        /* Idle state — treat byte directly as a button code */
        _ProcessButtonByte(b);
    }

    HAL_UART_Receive_IT(_huart, &_rx_byte, 1);
}

/* ============================================================
 * Public API — State Getters
 * ============================================================ */

const JoyState_t *JOY_GetState(void) { return &_state; }

JoyButton_t JOY_GetButtons(void) { return _state.buttons; }

JoyAxis_t JOY_GetLeftAxis(void) {
    return (JoyAxis_t) {_state.left.x, _state.left.y};
}

JoyAxis_t JOY_GetRightAxis(void) {
    return (JoyAxis_t) {_state.right.x, _state.right.y};
}

/* ============================================================
 * Public API — Axis Accessors
 * ============================================================ */

int16_t JOY_LeftX(void) { return _state.left.x; }

int16_t JOY_LeftY(void) { return _state.left.y; }

int16_t JOY_RightX(void) { return _state.right.x; }

int16_t JOY_RightY(void) { return _state.right.y; }

uint8_t JOY_LeftActive(int16_t deadzone) {
    return ((_state.left.x > deadzone || _state.left.x < -deadzone) ||
            (_state.left.y > deadzone || _state.left.y < -deadzone))
               ? 1
               : 0;
}

uint8_t JOY_RightActive(int16_t deadzone) {
    return ((_state.right.x > deadzone || _state.right.x < -deadzone) ||
            (_state.right.y > deadzone || _state.right.y < -deadzone))
               ? 1
               : 0;
}

int8_t JOY_AxisPercent(int16_t axis_val) {
    int32_t pct = ((int32_t)axis_val * 100) / 128;
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;
    return (int8_t)pct;
}

/* ============================================================
 * Public API — Button Helpers
 * ============================================================ */

uint8_t JOY_IsPressed(JoyButton_t btn) {
    return ((_state.buttons & btn) == btn) ? 1 : 0;
}

uint8_t JOY_IsAnyPressed(JoyButton_t btn) {
    return ((_state.buttons & btn) != 0) ? 1 : 0;
}

void JOY_ClearButton(JoyButton_t btn) {
    _active_dpad    &= ~btn;
    _active_actions &= ~btn;
    _state.buttons  &= ~btn;
    _SyncFlags();
}

void JOY_ClearAllButtons(void) {
    _active_dpad    = JOY_NONE;
    _active_actions = JOY_NONE;
    _state.buttons  = JOY_NONE;
    _SyncFlags();
}

/* ============================================================
 * Private Functions
 * ============================================================ */

static JoyButton_t _MapByte(uint8_t byte) {
    switch (byte) {
        /* D-Pad */
        case 0x41: return JOY_UP;
        case 0x42: return JOY_DOWN;
        case 0x43: return JOY_LEFT;
        case 0x44:
            return JOY_RIGHT;

            /* Face Buttons */
        case 0x4C: return JOY_CIRCLE;
        case 0x49: return JOY_TRIANGLE;
        case 0x4B: return JOY_SQUARE;
        case 0x4A:
            return JOY_XMARK;

            /* Menu */
        case 0x47: return JOY_SELECT;
        case 0x48:
            return JOY_START;

            /* Shoulder */
        case 0x45: return JOY_L1;
        case 0x46: return JOY_L2;
        case 0x4D: return JOY_R1;
        case 0x4E: return JOY_R2;

        default: return JOY_NONE;
    }
}

/* Raw 0-255 → signed centered: subtract center offset */
static inline int16_t _Remap(uint8_t raw, uint8_t center) {
    return (int16_t)raw - (int16_t)center;
}

static void _ParsePacket(void) {
    char *buf = (char *)_packet_buf;

    if (buf[0] == 'W') {
        char *p_ptr = strchr(buf, 'P');
        if (p_ptr) {
            _state.left.x = _Remap((uint8_t)atoi(buf + 1), 127);
            _state.left.y = _Remap((uint8_t)atoi(p_ptr + 1), 128);
        }
    } else if (buf[0] == 'Q') {
        char *s_ptr = strchr(buf, 'S');
        if (s_ptr) {
            _state.right.x = _Remap((uint8_t)atoi(buf + 1), 127);
            _state.right.y = _Remap((uint8_t)atoi(s_ptr + 1), 128);
        }
    }
}

/* Mirror bitmask → individual bool flags + joy_debug array */
static void _SyncFlags(void) {
    _state.btn_up       = (_state.buttons & JOY_UP) ? 1 : 0;
    _state.btn_down     = (_state.buttons & JOY_DOWN) ? 1 : 0;
    _state.btn_left     = (_state.buttons & JOY_LEFT) ? 1 : 0;
    _state.btn_right    = (_state.buttons & JOY_RIGHT) ? 1 : 0;
    _state.btn_circle   = (_state.buttons & JOY_CIRCLE) ? 1 : 0;
    _state.btn_triangle = (_state.buttons & JOY_TRIANGLE) ? 1 : 0;
    _state.btn_square   = (_state.buttons & JOY_SQUARE) ? 1 : 0;
    _state.btn_xmark    = (_state.buttons & JOY_XMARK) ? 1 : 0;
    _state.btn_select   = (_state.buttons & JOY_SELECT) ? 1 : 0;
    _state.btn_start    = (_state.buttons & JOY_START) ? 1 : 0;
    _state.btn_l1       = (_state.buttons & JOY_L1) ? 1 : 0;
    _state.btn_l2       = (_state.buttons & JOY_L2) ? 1 : 0;
    _state.btn_r1       = (_state.buttons & JOY_R1) ? 1 : 0;
    _state.btn_r2       = (_state.buttons & JOY_R2) ? 1 : 0;

    /* Debug mirror for Live Expression */
    joy_debug[0]  = _state.btn_up;
    joy_debug[1]  = _state.btn_down;
    joy_debug[2]  = _state.btn_left;
    joy_debug[3]  = _state.btn_right;
    joy_debug[4]  = _state.btn_circle;
    joy_debug[5]  = _state.btn_triangle;
    joy_debug[6]  = _state.btn_square;
    joy_debug[7]  = _state.btn_xmark;
    joy_debug[8]  = _state.btn_select;
    joy_debug[9]  = _state.btn_start;
    joy_debug[10] = _state.btn_l1;
    joy_debug[11] = _state.btn_l2;
    joy_debug[12] = _state.btn_r1;
    joy_debug[13] = _state.btn_r2;
    /* [14] and [15] are updated live in JOY_RxCpltCallback */
}
