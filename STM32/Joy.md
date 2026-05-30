# joy — PS-Style Joystick Library for STM32

A UART-based driver for PS-style joystick controllers on STM32 (HAL). Handles simultaneous button bitmasks via categorised dual-timeouts, parses analog axes packets, and manages individual live expression flags — all completely driven by non-blocking UART receive interrupts.

---

## Quick Setup

### 1. Include the files
```c
#include "joy.h"
```

### 2. Initialize in `main()`
```c
JOY_Init(&huart3);   // pass your UART handle
```

### 3. Hook the ISR in `stm32g4xx_it.c` (or `main.c`)
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    JOY_RxCpltCallback(huart);
}
```

### 4. Call Update in your main loop
```c
while (1) {
    JOY_Update();   // handles independent timeouts for D-Pad and Action buttons
    // ... your logic
}
```

---

## Protocol & Timing Architecture

### UART Packet Layout
| Input | Format | Example | Description |
|---|---|---|---|
| **Button press** | Single raw byte | `0x43` | Sends character `'C'` for D-Pad Left |
| **Left analog** | `W<0-255>P<0-255>\r\n` | `W127P0\r\n` | Streamed continuously during movement |
| **Right analog** | `Q<0-255>S<0-255>\r\n` | `Q255S128\r\n` | Streamed continuously during movement |
| **Button release**| *Sends absolutely nothing* | — | Handled strictly by time-domain parsing |

### Independent Category Timeouts
Because the controller never sends a "release" packet, button releases must be assumed via timeouts. To prevent held D-pad directions from getting artificially "stuck" or continuously repeating when tapping face buttons, the library internally processes state using **Independent Category Timeouts**:

* **D-Pad Mask (`_active_dpad`):** Refreshes its timeout tick *only* when a new D-pad byte arrives.
* **Action Mask (`_active_actions`):** Accumulates buttons and refreshes its timeout tick *only* when face, menu, or shoulder buttons arrive.

This prevents face button interaction from resetting the lifetime of an already physically released D-pad direction.

---

## API Reference

### Core Functions

#### `void JOY_Init(UART_HandleTypeDef *huart)`
Initializes the library state, zeros out all data structures, and registers the initial 1-byte interrupt layout.

| Parameter | Type | Description |
|---|---|---|
| `huart` | `UART_HandleTypeDef *` | Pointer to the initialized HAL UART handle connected to the joystick (e.g. `&huart3`) |

---

#### `void JOY_RxCpltCallback(UART_HandleTypeDef *huart)`
Processes each incoming byte dynamically inside the ISR context. Reassembles ASCII analog streams or maps single button bytes, then schedules subsequent single-byte interrupt receipts.

| Parameter | Type | Description |
|---|---|---|
| `huart` | `UART_HandleTypeDef *` | The UART handle passed by HAL — validated internally against the registered initialization handle |

---

#### `void JOY_Update(void)`
Evaluates the elapsed system time (`HAL_GetTick()`) against the individual D-pad and Action categories. Clears out stale button masks independently and updates internal state maps. **Must be called once per main loop iteration.**

No parameters.

---

### Button Functions

#### `uint8_t JOY_IsPressed(JoyButton_t btn)`
Returns `1` if **all** targeted bits in the bitmask are currently asserted simultaneously.

| Parameter | Type | Description |
|---|---|---|
| `btn` | `JoyButton_t` | Combined bitfield layout to test against (e.g., `JOY_L1 \| JOY_R1`) |

---

#### `uint8_t JOY_IsAnyPressed(JoyButton_t btn)`
Returns `1` if **any** of the queried bits in the bitmask are active.

| Parameter | Type | Description |
|---|---|---|
| `btn` | `JoyButton_t` | Combined bitfield layout (e.g., `JOY_UP \| JOY_DOWN`) |

---

#### `void JOY_ClearButton(JoyButton_t btn)`
Manually clears specific buttons out of both the global state mask and their localized category tracker masks.

| Parameter | Type | Description |
|---|---|---|
| `btn` | `JoyButton_t` | Button target or combined targets to strip out |

---

#### `void JOY_ClearAllButtons(void)`
Completely flushes all tracking categories, global bitmasks, and debug visibility registers back to zero.

---

#### `JoyButton_t JOY_GetButtons(void)`
Returns the raw `uint16_t` snapshot bitmask of all combined active buttons.

---

### Analog Axis Functions

Axes are automatically remapped from raw `0..255` packets down to signed integers perfectly centered at 0:
* **X-Axis:** `-127` (Full Left) to `+127` (Full Right)
* **Y-Axis:** `-128` (Full Up) to `+128` (Full Down)

#### `int16_t JOY_LeftX(void)` / `int16_t JOY_LeftY(void)`
#### `int16_t JOY_RightX(void)` / `int16_t JOY_RightY(void)`
Returns individual remapped axis readings.

---

#### `uint8_t JOY_LeftActive(int16_t deadzone)`
#### `uint8_t JOY_RightActive(int16_t deadzone)`
Checks if the absolute position of the requested thumbstick has broken past a user-specified radius barrier. Returns `1` if active, `0` if drifting or resting.

---

#### `int8_t JOY_AxisPercent(int16_t axis_val)`
Converts any processed axis variable into a standard percentage mapping ranging from **-100 to +100**.

---

#### `const JoyState_t* JOY_GetState(void)`
Provides a direct memory-mapped read-only pointer to the underlying master `JoyState_t` structure.

---

## Live Expression / Debugger Watch Variables

Add these exact names into your STM32CubeIDE **Live Expressions** panel to safely watch the system registers update in real-time while your software runs:

### System State Triggers
| Expression | Data Type | Internal Description |
|---|---|---|
| `_state.buttons` | `uint16_t` (Hex format) | Master runtime bitmask merging all active categories |
| `_active_dpad` | `uint16_t` (Hex format) | Standalone D-Pad filter tracking current raw navigation state |
| `_active_actions` | `uint16_t` (Hex format) | Standalone Action filter tracking current face/shoulder inputs |
| `_last_dpad_tick` | `uint32_t` | Millisecond uptime marker of the last valid D-Pad transmission |
| `_last_action_tick` | `uint32_t` | Millisecond uptime marker of the last valid Action transmission |

### Boolean Flag Verification
| Expression | Value (0 or 1) | Watch Target |
|---|---|---|
| `_state.btn_up` | `uint8_t` | D-pad Up |
| `_state.btn_down` | `uint8_t` | D-pad Down |
| `_state.btn_left` | `uint8_t` | D-pad Left |
| `_state.btn_right` | `uint8_t` | D-pad Right |
| `_state.btn_circle`| `uint8_t` | **Circle (○) Button [Fixed Map]** |
| `_state.btn_triangle`| `uint8_t` | Triangle (△) Button |
| `_state.btn_square`| `uint8_t` | Square (□) Button |
| `_state.btn_xmark` | `uint8_t` | Cross (✕) Button |

---

## Full Usage Example

```c
#include "joy.h"

// In main() after peripheral setup:
JOY_Init(&huart3);

while (1) {
    JOY_Update();   // Must execute frequently to resolve timeouts

    // --- Process Sticks via Percentage Remaps ---
    if (JOY_LeftActive(12)) {
        int8_t drive_speed = JOY_AxisPercent(JOY_LeftY());  // -100 to +100
        int8_t steer_angle = JOY_AxisPercent(JOY_LeftX());  // -100 to +100
        SetMotorOutputs(drive_speed, steer_angle);
    }

    // --- Isolated Pulse / Trigger Handling ---
    if (JOY_IsPressed(JOY_CIRCLE)) {
        FireActuator();
        JOY_ClearButton(JOY_CIRCLE);  // Flushes layout immediately until next event
    }

    // --- Synchronous Safety Combinations ---
    if (JOY_IsPressed(JOY_L1 | JOY_R1)) {
        KillSystemPower(); // Fires only when BOTH are held together
    }
}
```
