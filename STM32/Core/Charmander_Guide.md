# Charmander — Modbus RTU Command Bridge (STM32)

Charmander is a Modbus RTU slave library for STM32 that connects a PC (master) to robot control logic.

It handles:
- FC06 (Write Single Register)
- FC03 (Read Holding Registers)
- Heartbeat (YA / HI)

---

# 📦 Setup

## 1. Include Library

```c
#include "charmander.h"
```

---

## 2. Initialize

Call once after HAL init:

```c
Charmander_Init();
```

---

## 3. UART RX Interrupt Hook

### Global variable:

```c
uint8_t rx_byte;
```

### Start interrupt:

```c
HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
```

### Callback:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        Charmander_Feed(rx_byte);
        HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);
    }
}
```

---

## 4. Main Loop

```c
while (1)
{
    Charmander_Process();
    Charmander_Tick();

    // Your robot logic here
}
```

---

# 📥 Reading Commands (PC → Robot)

All decoded values are stored in:

```c
charmander
```

### Example

```c
if (strcmp(charmander.mode, "JOG") == 0)
{
    int16_t move = charmander.jog_degrees;
}
```

---

# 📤 Writing Robot State (Robot → PC)

You MUST update these before PC reads:

```c
Charmander_SetSensors(sensor_bits);
Charmander_SetTask(task_bits);
Charmander_SetMotion(position, velocity, acceleration);
Charmander_SetEmergency(active);
```

---

### Example

```c
Charmander_SetMotion(90.0f, 120.0f, 300.0f);
Charmander_SetSensors(0x0003);
Charmander_SetTask(0x0002);
Charmander_SetEmergency(0);
```

---

# ❤️ Heartbeat

| State   | Meaning |
|--------|--------|
| WAITING | No connection yet |
| ALIVE   | PC responding |
| DEAD    | Timeout (>2s) |

Handled automatically via:

```c
Charmander_Tick();
```

---

# ⚠️ Important Rule

❌ DO NOT CALL:

```c
Charmander_SendHeartbeat();
```

Modbus slaves must NOT transmit without request.

---

# 🧪 Live Expressions (STM32CubeIDE)

Add these to **Live Expressions** for real-time debugging:

---

## 🔹 Core Status

```c
charmander.heartbeat
charmander.mode
charmander.soft_stop
```

---

## 🔹 Heartbeat Debug

```c
charmander.hb_rx_status
charmander.hb_tx_status
charmander.hb_last_alive_ms
charmander.hb_hi_recv_count
charmander.hb_ya_sent_count
```

---

## 🔹 Motion

```c
charmander.position_real
charmander.velocity_real
charmander.accel_real
```

---

## 🔹 Sensors & Task

```c
charmander.sensor_raw
charmander.sensor_vertical
charmander.sensor_jaw

charmander.task_raw
charmander.task
```

---

## 🔹 Commands from PC

```c
charmander.mode_raw
charmander.jog_degrees
charmander.gripper_vertical
charmander.gripper_jaw
charmander.gripper_seq
```

---

## 🔹 Debug Counters

```c
charmander.frame_count
charmander.crc_error_count
charmander.read_req_count

charmander.last_reg_addr
charmander.last_reg_value
```

---

# 🧩 Minimal Working Example

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_LPUART1_UART_Init();

    uint8_t rx_byte;
    HAL_UART_Receive_IT(&hlpuart1, &rx_byte, 1);

    Charmander_Init();

    while (1)
    {
        Charmander_Process();
        Charmander_Tick();

        Charmander_SetMotion(10.0f, 5.0f, 2.0f);
        Charmander_SetSensors(0x0001);
        Charmander_SetTask(0x0001);
    }
}
```

---

# 🔁 Data Flow

```
PC (Master)
   ↓ Write / Read
Charmander (STM32)
   ↓ updates struct
Your Logic
   ↓ updates state
Charmander
   ↓ sends reply
PC
```

---

# ✅ Summary

- Feed bytes in ISR
- Process in main loop
- Update robot state manually
- PC controls everything
- Heartbeat is automatic
