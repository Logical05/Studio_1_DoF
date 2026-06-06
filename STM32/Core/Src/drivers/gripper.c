#include "drivers/gripper.h"

/* ============================================================================
 * Global
 * ========================================================================== */

Gripper_t gripper = {0};

/* ============================================================================
 * Internal
 * ========================================================================== */

// static void Gripper_StopOutputs(void) {
//     HAL_GPIO_WritePin(GRIPPER_OPEN_OUT_PORT, GRIPPER_OPEN_OUT_PIN, GRIPPER_OFF);
//     HAL_GPIO_WritePin(GRIPPER_CLOSE_OUT_PORT, GRIPPER_CLOSE_OUT_PIN, GRIPPER_OFF);
//     HAL_GPIO_WritePin(GRIPPER_UP_OUT_PORT, GRIPPER_UP_OUT_PIN, GRIPPER_OFF);
//     HAL_GPIO_WritePin(GRIPPER_DOWN_OUT_PORT, GRIPPER_DOWN_OUT_PIN, GRIPPER_OFF);
// }

static void Gripper_Open(void) {
    HAL_GPIO_WritePin(GRIPPER_OPEN_OUT_PORT, GRIPPER_OPEN_OUT_PIN, GRIPPER_ON);
    HAL_GPIO_WritePin(GRIPPER_CLOSE_OUT_PORT, GRIPPER_CLOSE_OUT_PIN, GRIPPER_OFF);
    gripper.gripper_is_closed = false;
}

static void Gripper_Close(void) {
    HAL_GPIO_WritePin(GRIPPER_CLOSE_OUT_PORT, GRIPPER_CLOSE_OUT_PIN, GRIPPER_ON);
    HAL_GPIO_WritePin(GRIPPER_OPEN_OUT_PORT, GRIPPER_OPEN_OUT_PIN, GRIPPER_OFF);
    gripper.gripper_is_closed = true;
}

static void Gripper_MoveUp(void) {
    HAL_GPIO_WritePin(GRIPPER_UP_OUT_PORT, GRIPPER_UP_OUT_PIN, GRIPPER_ON);
    HAL_GPIO_WritePin(GRIPPER_DOWN_OUT_PORT, GRIPPER_DOWN_OUT_PIN, GRIPPER_OFF);
    gripper.gripper_is_down = false;
}

static void Gripper_MoveDown(void) {
    HAL_GPIO_WritePin(GRIPPER_DOWN_OUT_PORT, GRIPPER_DOWN_OUT_PIN, GRIPPER_ON);
    HAL_GPIO_WritePin(GRIPPER_UP_OUT_PORT, GRIPPER_UP_OUT_PIN, GRIPPER_OFF);
    gripper.gripper_is_down = true;
}

/* ============================================================================
 * Reed Status
 * ========================================================================== */
bool Gripper_IsOpened(void) { return !Gripper_IsClosed(); }

bool Gripper_IsClosed(void) { return gripper.gripper_is_closed; }

bool Gripper_IsUp(void) {
    return HAL_GPIO_ReadPin(GRIPPER_REED_UP_PORT, GRIPPER_REED_UP_PIN) ==
           REED_ACTIVE_STATE;
}

bool Gripper_IsDown(void) {
    return HAL_GPIO_ReadPin(GRIPPER_REED_DOWN_PORT, GRIPPER_REED_DOWN_PIN) ==
           REED_ACTIVE_STATE;
}

/* ============================================================================
 * Init
 * ========================================================================== */

void Gripper_Init(void) {
    Gripper_Open();
    Gripper_MoveUp();

    gripper.state             = GRIPPER_IDLE;
    gripper.busy              = false;
    gripper.gripper_is_closed = false;
    gripper.gripper_is_down   = false;
}

/* ============================================================================
 * Command
 * ========================================================================== */
bool Gripper_Command(GripperCommand_t cmd) {
    if (gripper.busy) {
        return false;
    }

    // Gripper_StopOutputs();

    gripper.busy = true;

    gripper.cmd  = cmd;
    gripper.tick = HAL_GetTick();

    switch (cmd) {

            /* =========================================================
             * LOW LEVEL
             * =======================================================*/

        case GRIPPER_CMD_OPEN:
            Gripper_Open();
            gripper.state = GRIPPER_OPENING;
            break;
        case GRIPPER_CMD_CLOSE:
            Gripper_Close();
            gripper.state = GRIPPER_CLOSING;
            break;
        case GRIPPER_CMD_UP:
            Gripper_MoveUp();
            gripper.state = GRIPPER_MOVING_UP;
            break;
        case GRIPPER_CMD_DOWN:
            Gripper_MoveDown();
            gripper.state = GRIPPER_MOVING_DOWN;
            break;

            /* =========================================================
             * HIGH LEVEL
             * =======================================================*/

        case GRIPPER_CMD_PICK:
            Gripper_MoveDown();
            gripper.state = GRIPPER_PICK_DOWN;
            break;

        case GRIPPER_CMD_PLACE:
            Gripper_MoveDown();
            gripper.state = GRIPPER_PLACE_DOWN;
            break;
        default: gripper.busy = false; return false;
    }
    return true;
}

/* ============================================================================
 * Update
 * ========================================================================== */
void Gripper_Update(void) {
    if (!gripper.busy) {
        return;
    }

    /* =========================================================
     * TIMEOUT
     * =======================================================*/

    if ((HAL_GetTick() - gripper.tick) >= GRIPPER_TIMEOUT_MS) {
        // Gripper_StopOutputs();
        gripper.state = GRIPPER_TIMEOUT;
        gripper.busy  = false;
        return;
    }

    switch (gripper.state) {

            /* =====================================================
             * LOW LEVEL
             * ===================================================*/

        case GRIPPER_OPENING:
            if ((HAL_GetTick() - gripper.tick) >= GRIPPER_TIMEOUT_MS / 4) {
                // Gripper_StopOutputs();
                gripper.state = GRIPPER_SUCCESS;
                gripper.busy  = false;
            }
            break;
        case GRIPPER_CLOSING:
            if ((HAL_GetTick() - gripper.tick) >= GRIPPER_TIMEOUT_MS / 4) {
                // Gripper_StopOutputs();
                gripper.state = GRIPPER_SUCCESS;
                gripper.busy  = false;
            }
            break;
        case GRIPPER_MOVING_UP:
            if (Gripper_IsUp()) {
                // Gripper_StopOutputs();
                gripper.state = GRIPPER_SUCCESS;
                gripper.busy  = false;
            }
            break;
        case GRIPPER_MOVING_DOWN:
            if (Gripper_IsDown()) {
                // Gripper_StopOutputs();
                gripper.state = GRIPPER_SUCCESS;
                gripper.busy  = false;
            }
            break;

            /* =====================================================
             * PICK SEQUENCE
             * ===================================================*/

        case GRIPPER_PICK_DOWN:
            if (Gripper_IsDown()) {
                // Gripper_StopOutputs();
                Gripper_Close();
                gripper.state = GRIPPER_PICK_CLOSE;
            }
            break;
        case GRIPPER_PICK_CLOSE:
            if ((HAL_GetTick() - gripper.tick) >= GRIPPER_TIMEOUT_MS / 4) {
                // Gripper_StopOutputs();
                Gripper_MoveUp();
                gripper.state = GRIPPER_PICK_UP;
            }
            break;
        case GRIPPER_PICK_UP:
            if (Gripper_IsUp()) {
                // Gripper_StopOutputs();
                gripper.state = GRIPPER_SUCCESS;
                gripper.busy  = false;
            }
            break;

            /* =====================================================
             * PLACE SEQUENCE
             * ===================================================*/

        case GRIPPER_PLACE_DOWN:
            if (Gripper_IsDown()) {
                // Gripper_StopOutputs();
                Gripper_Open();
                gripper.state = GRIPPER_PLACE_OPEN;
            }
            break;
        case GRIPPER_PLACE_OPEN:
            if ((HAL_GetTick() - gripper.tick) >= GRIPPER_TIMEOUT_MS / 4) {
                // Gripper_StopOutputs();
                Gripper_MoveUp();
                gripper.state = GRIPPER_PLACE_UP;
            }
            break;
        case GRIPPER_PLACE_UP:
            if (Gripper_IsUp()) {
                // Gripper_StopOutputs();
                gripper.state = GRIPPER_SUCCESS;
                gripper.busy  = false;
            }
            break;
        default: break;
    }
}

/* ============================================================================
 * Busy
 * ========================================================================== */

bool Gripper_IsBusy(void) { return gripper.busy; }
