/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "pid.h"
#include "trajectory.h"
#include "useful.h"
#include "kalman.h"
#include "charmander.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	QEI_NOW, QEI_PREV
} QEI_State;

typedef enum {
	Position, Velocity
} Reference_State;

typedef struct {
	Pinout_TypeDef SSR_TRIG;
	Pinout_TypeDef Motor_DIR;
	Pinout_TypeDef Relay_IN;
	Pinout_TypeDef Reset_5W_IN;
	Pinout_TypeDef PROX_IN;
} PIN_TypeDef;

typedef struct {
	uint32_t position[2];
	float theta; /* [rad] cumulative encoder angle */
	float omega; /* [rad/s] raw                    */
	float filter_omega; /* [rad/s] Kalman-filtered        */
} QEI_TypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define INRANGE(v, r) ((-r <= v) && (v <= r))
#define INDEX_TO_RAD(i) DEG_TO_RAD((float)(i) * 5.0f)	// 1 slots → 5°/slot

#define ONE_TURN_QEI_CNT 20480
#define MULTI_TURN_QEI_CNT 61440
#define OBSERVER_PERIOD 0.001 // 1kHz
#define TRAJ_PERIOD 0.002 // 500Hz

#define HOMING_VOLTAGE (-3.0f)   /* V — slow CW creep; adjust sign/magnitude */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ── GPIO pin map ────────────────────────────────────────────────────────── */
const PIN_TypeDef GPIO = { { GPIOA, GPIO_PIN_15 }, /* SSR_TRIG    */
{ GPIOB, GPIO_PIN_11 }, /* Motor_DIR   */
{ GPIOC, GPIO_PIN_12 }, /* Relay_IN    */
{ GPIOC, GPIO_PIN_11 }, /* Reset_5W_IN */
{ GPIOC, GPIO_PIN_3 }, /* PROX_IN     */
};

/* ── Sensor / motion objects ─────────────────────────────────────────────── */
QEI_TypeDef QEIdata = { 0 };
KF_TypeDef kf;

PID_TypeDef PD_Pos = { 0 }; /* outer position loop (PD)  */
PID_TypeDef PI_Velo = { 0 }; /* inner velocity loop (PI)  */

MJT_Trajectory traj = { 0 };
float Reference[2] = { 0.0f, 0.0f }; /* [Position], [Velocity] */

DelayTimer wait_timer; /* generic one-shot delay used by sequences */

/* ── Tuning knobs (changeable without recompile via debugger) ─────────────── */
float KF_Q = 1.0e-3f;
float VMAX = 5.31f; /* [rad/s] trajectory speed limit            */

/* ── Safety flags ────────────────────────────────────────────────────────── */
bool EMERGENCY = true;
bool emergency_btn = false; /* live GPIO level of the E-stop button      */
bool emergency_latched = true; /* sticky latch; cleared only by RESET press */

/* ── UART byte buffer for interrupt-driven Modbus reception ──────────────── */
static uint8_t lpuart1_rx_byte;

/* ── Mode-local state variables ──────────────────────────────────────────── */

/* HOME mode */
bool homing_active = false;

/* AUTO / pick-place mode */
static uint16_t pp_current_pair = 0; /* which pair we are executing (0-based) */

/* TEST mode */
static float prec_pts[64];
static bool test_running = false;
static bool init_reached = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* ── Hardware helpers ────────────────────────────────────────── */
void Init(void);
void Break(void);
void Steerinng(float voltage);
void QEI_Reset(void);

/* ── Charmander bridge ───────────────────────────────────────── */
void Charmander_Sync_State(void);
void Charmander_Update_Feedback(void);

/* ── Mode handlers (one per CharmanderMode_t value) ─────────── */
void Mode_Idle(void);
void Mode_Home(void);
void Mode_Jog(void);
void Mode_Auto(void);
void Mode_P2P(void);
void Mode_SetHome(void);
void Mode_Test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM16_Init();
	MX_TIM6_Init();
	MX_TIM8_Init();
	MX_USART1_UART_Init();
	MX_LPUART1_UART_Init();
	MX_TIM7_Init();
	/* USER CODE BEGIN 2 */
	Init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* 1. Parse any Modbus bytes that arrived since last loop */
		Charmander_Process();

		/* 2. Run heartbeat timeout monitor */
		Charmander_Tick();

		/* 3. Sync hardware buttons + Modbus fields → local state */
		Charmander_Sync_State();

		/* 4. Mirror motion feedback into the Modbus READ registers */
		Charmander_Update_Feedback();

		/* 5. Safety gate — motor must not move while EMERGENCY is set */
		if (EMERGENCY) {
			Break();
			Mode_Idle();
			charmander.mode = CHARMANDER_MODE_IDLE;
			continue;
		}

		/* 6. Mode dispatch — driven entirely by charmander.mode */
		switch (charmander.mode) {
			case CHARMANDER_MODE_IDLE:
				Mode_Idle();
				break;
			case CHARMANDER_MODE_HOME:
				Mode_Home();
				break;
			case CHARMANDER_MODE_JOG:
				Mode_Jog();
				break;
			case CHARMANDER_MODE_AUTO:
				Mode_Auto();
				break;
			case CHARMANDER_MODE_P2P:
				Mode_P2P();
				break;
			case CHARMANDER_MODE_SET_HOME:
				Mode_SetHome();
				break;
			case CHARMANDER_MODE_TEST:
				Mode_Test();
				break;
			default:
				Mode_Idle();
				break;
		}

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); /* heartbeat LED */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
									| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* ============================================================================
 *  Init
 * ============================================================================ */
void Init(void) {
	Break();

	HAL_GPIO_WritePin(GPIO.SSR_TRIG.GPIOx, GPIO.SSR_TRIG.GPIO_Pin, GPIO_PIN_RESET);

	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

	/* ── PID: outer position loop (PD) ──────────────────────────────── */
	PID_Init(&PD_Pos, 3.0f, 0.0f, 0.12f, 5.31f, false, 0.0f);

	/* ── PID: inner velocity loop (PI) ──────────────────────────────── */
	PID_Init(&PI_Velo, 6.7f, 0.093f, 0.0f, 24.0f, true, 12.0f);

	/* ── Kalman filter (DC motor physical parameters) ────────────────── */
	KF_DCMotor_Init(&kf, OBSERVER_PERIOD, /* dt                     */
	0.029842f, /* J  [kg·m²]             */
	0.78891f, /* B  [N·m·s/rad]         */
	2.2321f, /* Kt [N·m/A]             */
	2.2321f, /* Ke [V·s/rad]           */
	9.0473919f, /* Rm [Ω]                 */
	0.0017419f, /* L  [H]                 */
	KF_Q, /* Q  (process noise)     */
	5e-5f /* R  (measurement noise) */
	);

	/* ── Charmander (Modbus RTU slave) ───────────────────────────────── */
	Charmander_Init();
	HAL_UART_Receive_IT(&hlpuart1, &lpuart1_rx_byte, 1);
}

/* ============================================================================
 *  QEI helpers
 * ============================================================================ */
void QEI_Reset(void) {
	QEIdata.theta = 0.0f;
	QEIdata.omega = 0.0f;
	QEIdata.filter_omega = 0.0f;
	QEIdata.position[QEI_NOW] = __HAL_TIM_GET_COUNTER(&htim8);
	QEIdata.position[QEI_PREV] = QEIdata.position[QEI_NOW];
}

/* ============================================================================
 *  Break / Steerinng
 * ============================================================================ */
void Break(void) {
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
	HAL_GPIO_WritePin(GPIO.Motor_DIR.GPIOx, GPIO.Motor_DIR.GPIO_Pin, GPIO_PIN_RESET);
}

void Steerinng(float voltage) {
	if (voltage == 0.0f) {
		Break();
		return;
	}

	HAL_GPIO_WritePin(GPIO.Motor_DIR.GPIOx, GPIO.Motor_DIR.GPIO_Pin,
		voltage > 0.0f ? GPIO_PIN_RESET : GPIO_PIN_SET);

	uint16_t pwm = (uint16_t) map(fabsf(voltage), 0.0f, 24.0f, 0.0f, 100.0f);
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwm);
}

/* ============================================================================
 *  Charmander_Sync_State
 *  Read physical buttons and Modbus fields → update local safety flags.
 * ============================================================================ */
void Charmander_Sync_State(void) {
	/* ── Physical EMERGENCY button (Relay_IN, active-high) ──────────── */
	emergency_btn = (HAL_GPIO_ReadPin(GPIO.Relay_IN.GPIOx, GPIO.Relay_IN.GPIO_Pin)
			== GPIO_PIN_SET);
	if (emergency_btn) emergency_latched = true;

	/* ── Composite EMERGENCY: button | latch | link dead ─────────────── */
	bool link_dead = (charmander.heartbeat != CHARMANDER_HB_ALIVE);
	EMERGENCY = emergency_btn || emergency_latched || link_dead;

	/* Soft-stop from PC (reg 0x25) also triggers EMERGENCY */
	if (charmander.soft_stop == CHARMANDER_STOP_ACTIVE) {
		EMERGENCY = true;
	}
}

/* ============================================================================
 *  Charmander_Update_Feedback
 *  Push latest motion data into the Modbus READ-side registers.
 * ============================================================================ */
void Charmander_Update_Feedback(void) {
	float pos_deg = RAD_TO_DEG(QEIdata.theta);
	float vel_dps = RAD_TO_DEG(QEIdata.filter_omega);
	float accel_dps2 = 0.0f; /* replace with real derivative when available */

	Charmander_SetMotion(pos_deg, vel_dps, accel_dps2);
	Charmander_SetEmergency(EMERGENCY ? 1 : 0);
}

/* ============================================================================
 *  MODE HANDLERS
 *  Each function handles exactly one CharmanderMode_t value.
 *  They read from charmander.* and touch nothing else directly.
 * ============================================================================ */

/* ── IDLE ────────────────────────────────────────────────────────────────── */
/*
 * No motion command active. Hold position at whatever theta currently is,
 * or simply brake. We keep Reference unchanged so the PID loops stay
 * commanded at the last known good position.
 */
void Mode_Idle(void) {
	Charmander_SetTask(0x0000); /* IDLE */

	/* Re-arm reference to the current position so we hold it */
	Reference[Position] = QEIdata.theta;
	Reference[Velocity] = 0.0f;

	/* Reset mode-local state so next mode entry starts clean */
	homing_active = false;
	pp_current_pair = 0;
	test_running = false;
}

/* ── HOME ────────────────────────────────────────────────────────────────── */
/*
 * Creep motor at HOMING_VOLTAGE until the proximity sensor fires.
 * Once fired: zero the encoder, reset PIDs, transition back to IDLE.
 * Task feedback: bit 0 of reg 0x27 → HOMING.
 */
void Mode_Home(void) {
	Charmander_SetTask(0x0001); /* 0x27 bit 0 = Homing */

	GPIO_PinState prox = HAL_GPIO_ReadPin(GPIO.PROX_IN.GPIOx, GPIO.PROX_IN.GPIO_Pin);

	if (prox == GPIO_PIN_SET) {
		/* ── Prox fired: home found ───────────────────────────────── */
		Break();
		charmander.mode = CHARMANDER_MODE_SET_HOME;
		/* Mode stays HOME until PC writes a new mode — that is fine; the
		 * motor simply holds position at theta=0. */
	} else {
		/* ── Creep toward home ────────────────────────────────────── */
		homing_active = true;
		Steerinng(HOMING_VOLTAGE);
	}
}

/* ── JOG ─────────────────────────────────────────────────────────────────── */
/*
 * Move by the signed step written in charmander.jog_degrees.
 * Positive → CCW, Negative → CW (per register map §3.6).
 * A new jog is triggered only when jog_degrees changes value.
 * The trajectory runs one MJT segment; when done the axis holds position.
 */
void Mode_Jog(void) {
	/* New jog request from PC */
	if (charmander.jog_degrees != 0 && charmander.task == CHARMANDER_TASK_IDLE) {
		Charmander_SetTask(0x0008); /* Go Point */

		float step_rad = DEG_TO_RAD((float ) charmander.jog_degrees);
		float target = QEIdata.theta + step_rad;

		static float jog_pts[1];
		jog_pts[0] = target;
		MJT_Goal(&traj, jog_pts, 1, QEIdata.theta, VMAX / 3.0f);
	}

	/* Run the active trajectory segment */
	if (!MJT_is_Finished(&traj)) {
		if (traj.state == MJT_WAIT) MJT_Continue(&traj);

		Reference[Position] = MJT_get_Pos(&traj);
		Reference[Velocity] = MJT_get_Vel(&traj);
	} else {
		charmander.mode = CHARMANDER_MODE_IDLE;
		MJT_Reset(&traj);
	}
}

/* ── AUTO (pick-and-place sequence) ──────────────────────────────────────── */
/*
 * Executes the pick-place slot sequence stored in charmander.pp_slots[].
 * Slots come in pairs: slot[2n] = pick position, slot[2n+1] = place position.
 * charmander.pp_pair_count says how many pairs to run.
 *
 * Gripper auto-control is gated on charmander.gripper_auto == CHARMANDER_ENABLE.
 *
 * Task feedback:
 *   0x27 bit 1 = GO_PICK
 *   0x27 bit 2 = GO_PLACE
 */
void Mode_Auto(void) {
	uint16_t targets = charmander.pp_pair_count * 2;

	if (targets == 0) {
		charmander.mode = CHARMANDER_MODE_IDLE;
		return;
	}

	static float auto_pts[16] = { 0.0f };
	switch (traj.state) {
		case MJT_IDLE:
			for (int i = 0; i < targets; ++i) {
				int16_t slot = charmander.pp_slots[i];

				bool cw = (slot < 0);
				float target = INDEX_TO_RAD(abs(slot));
				float now = fmodf(QEIdata.theta, M_2PI);
				if (now < 0) now += M_2PI;
				if (cw && target > now) {
					target -= M_2PI;
				} else if (target < now) {
					target += M_2PI;
				}

				auto_pts[i] = QEIdata.theta + (target - now);
			}

			MJT_Goal(&traj, auto_pts, targets, QEIdata.theta, VMAX);
			Charmander_SetTask(0x0002);
			break;
		case MJT_WAIT:
			// Wait until pick/place done

			MJT_Continue(&traj);
			break;
		case MJT_DONE:
			charmander.mode = CHARMANDER_MODE_IDLE;

			MJT_Reset(&traj);
			return;
		default:
			break;
	}

	Reference[Position] = MJT_get_Pos(&traj);
	Reference[Velocity] = MJT_get_Vel(&traj);
}

void Mode_P2P(void) {
	static float p2p_pts[1] = { 0.0f };
	switch (traj.state) {
		case MJT_IDLE:
			int16_t target = charmander.p2p_target;
			bool cw = (target < 0);

			float rad =
					charmander.p2p_unit == CHARMANDER_UNIT_INDEX ?
							INDEX_TO_RAD(abs(target)) : DEG_TO_RAD(abs(target));

			float now = fmodf(QEIdata.theta, M_2PI);
			if (now < 0) now += M_2PI;
			if (cw && rad > now) {
				rad -= M_2PI;
			} else if (rad < now) {
				rad += M_2PI;
			}

			p2p_pts[0] = QEIdata.theta + (rad - now);

			MJT_Goal(&traj, p2p_pts, 1, QEIdata.theta, VMAX);
			Charmander_SetTask(0x0008); /* Go Point */
			break;
		case MJT_WAIT:
			MJT_Continue(&traj);
			break;
		case MJT_DONE:
			charmander.mode = CHARMANDER_MODE_IDLE;
			MJT_Reset(&traj);
			return;
		default:
			break;
	}

	Reference[Position] = MJT_get_Pos(&traj);
	Reference[Velocity] = MJT_get_Vel(&traj);
}

/* ── SET_HOME ────────────────────────────────────────────────────────────── */
/*
 * Capture current position as the new home (θ = 0) without moving.
 * Useful for soft-homing without a prox sensor.
 */
void Mode_SetHome(void) {
	QEI_Reset();
	PID_Reset(&PD_Pos);
	PID_Reset(&PI_Velo);

	Reference[Position] = 0.0f;
	Reference[Velocity] = 0.0f;

	Charmander_SetTask(0x0000);
	charmander.mode = CHARMANDER_MODE_IDLE;
	/* The PC should write a different mode after SET_HOME.
	 * We stay here holding theta=0 until it does. */
}

/* ── TEST ────────────────────────────────────────────────────────────────── */
/*
 * Two sub-modes selected by charmander.test_type:
 *
 *  PRECISION   — move back and forth between prec_init_pos and prec_final_pos,
 *                repeating prec_repeat_count times.
 *                Sign of prec_repeat_count selects unit (+ = degree, − = index).
 *
 *  PERFORMANCE — run trajectory at the velocity/acceleration specified in
 *                perf_velocity / perf_acceleration between two fixed points.
 *                Loops indefinitely until mode changes.
 */
void Mode_Test(void) {
	Charmander_SetTask(0x0000);

	if (charmander.test_type == CHARMANDER_TEST_PRECISION) {
		/* ── Precision test ───────────────────────────────────────── */

		float pos_init = 0.0f;
		float pos_final = 0.0f;
		if (charmander.prec_repeat_count > 0) {
			pos_init = DEG_TO_RAD(charmander.prec_init_pos);
			pos_final = DEG_TO_RAD(charmander.prec_final_pos);
		} else {
			pos_init = INDEX_TO_RAD(charmander.prec_init_pos);
			pos_final = INDEX_TO_RAD(charmander.prec_final_pos);
		}

		/* ---------------------------------------------------------
		 * Move to initial position once
		 * --------------------------------------------------------- */
		if (!test_running) {
			prec_pts[0] = pos_init;
			MJT_Goal(&traj, prec_pts, 1, QEIdata.theta, VMAX);
			test_running = true;
			init_reached = false;
		}

		/* Continue segments */
		if (traj.state == MJT_WAIT) {
			MJT_Continue(&traj);
		}

		/* ---------------------------------------------------------
		 * Initial positioning completed
		 * --------------------------------------------------------- */
		if (MJT_is_Finished(&traj) && !init_reached) {
			init_reached = true;
			MJT_Reset(&traj);

			uint8_t repeat = abs(charmander.prec_repeat_count);
			for (int i = 0; i < repeat * 2; i += 2) {
				prec_pts[i] = pos_final;
				prec_pts[i + 1] = pos_init;
			}
			MJT_Goal(&traj, prec_pts, repeat * 2, QEIdata.theta, VMAX);
		}

		/* ---------------------------------------------------------
		 * Precision test completed
		 * --------------------------------------------------------- */
		else if (MJT_is_Finished(&traj) && init_reached) {
			MJT_Reset(&traj);
			test_running = false;
			init_reached = false;

			charmander.mode = CHARMANDER_MODE_IDLE;
			return;
		}

		Reference[Position] = MJT_get_Pos(&traj);
		Reference[Velocity] = MJT_get_Vel(&traj);
	} else {
		float vmax_test = (float) charmander.perf_velocity;
		if (vmax_test < 0.1f) vmax_test = 0.1f;
		static float perf_pts[2] = { 0.0f, 0.0f };
		if (!test_running) {
			perf_pts[0] = DEG_TO_RAD(355.0f);
			perf_pts[1] = 0.0f;
			MJT_Goal(&traj, perf_pts, 2, QEIdata.theta, vmax_test);
			test_running = true;
		}
		if (traj.state == MJT_WAIT) MJT_Continue(&traj);
		if (MJT_is_Finished(&traj)) { /* Restart in opposite direction */
			float tmp = perf_pts[0];
			perf_pts[0] = perf_pts[1];
			perf_pts[1] = tmp;
			MJT_Goal(&traj, perf_pts, 2, QEIdata.theta, vmax_test);
		}
		Reference[Position] = MJT_get_Pos(&traj);
		Reference[Velocity] = MJT_get_Vel(&traj);
	}
}

/* ============================================================================
 *  Timer ISR callbacks
 * ============================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		QEIdata.position[QEI_NOW] = __HAL_TIM_GET_COUNTER(&htim8);

		int32_t diff = (int32_t) QEIdata.position[QEI_NOW]
				- (int32_t) QEIdata.position[QEI_PREV];

		if (diff > (MULTI_TURN_QEI_CNT / 2)) diff -= MULTI_TURN_QEI_CNT;
		else if (diff < -(MULTI_TURN_QEI_CNT / 2)) diff += MULTI_TURN_QEI_CNT;

		float d_theta = (float) diff * (M_2PI / ONE_TURN_QEI_CNT);
		QEIdata.theta += d_theta;
		QEIdata.omega = d_theta / OBSERVER_PERIOD;

		KF_SetQ(&kf, KF_Q, KF_Q * 10.0f, KF_Q * 10.0f, KF_Q * 100.0f);
		KF_Predict(&kf, PI_Velo.output);
		KF_Update(&kf, QEIdata.theta);

		QEIdata.filter_omega = kf.x[1];
		QEIdata.position[QEI_PREV] = QEIdata.position[QEI_NOW];

		if (EMERGENCY || charmander.mode == CHARMANDER_MODE_HOME) return; /* do not drive motor */

		PID_Calc(&PI_Velo, PD_Pos.output + Reference[Velocity],
			QEIdata.filter_omega);

		/* Deadband */
		if (INRANGE(PI_Velo.output, 0.05f)) PI_Velo.output = 0.0f;

		Steerinng(PI_Velo.output);
	}

	if (htim->Instance == TIM7) {
		MJT_Update(&traj, TRAJ_PERIOD);
		PID_Calc(&PD_Pos, Reference[Position], QEIdata.theta);
	}
}

/* ============================================================================
 *  UART Rx Complete — feed every byte into Charmander
 * ============================================================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == LPUART1) {
		Charmander_Feed(lpuart1_rx_byte);
		HAL_UART_Receive_IT(&hlpuart1, &lpuart1_rx_byte, 1);
	}
}

/* ============================================================================
 *  GPIO EXTI — RESET button
 * ============================================================================ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO.Reset_5W_IN.GPIO_Pin) {
		/*
		 * Clear the latch only when the E-stop button is NOT currently pressed.
		 * If the operator presses RESET while the E-stop is still held, ignore.
		 */
		if (!emergency_btn) {
			emergency_latched = false;
		}
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
