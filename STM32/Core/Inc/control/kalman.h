#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

/* =========================================================
 * 4-state Kalman filter for DC motor
 *
 * States:
 *   x[0]  theta      (rad)
 *   x[1]  omega      (rad/s)
 *   x[2]  current    (A)
 *   x[3]  load torque (Nm)  — random-walk disturbance
 *
 * Measurement:  encoder position (theta only)
 *   H = [1 0 0 0]
 *
 * Call order each cycle:
 *   1. KF_Predict(&kf, voltage);   ← uses previous voltage
 *   2. KF_Update (&kf, theta);     ← uses fresh encoder reading
 * ========================================================= */

typedef struct {
        /* --- State vector --- */
        float x[4];

        /* --- State transition matrix (4x4) --- */
        float F[4][4];

        /* --- Input matrix --- */
        float B[4];

        /* --- Observation matrix (row vector) --- */
        float H[4];

        /* --- Process noise covariance (4x4 diagonal) --- */
        float Q[4][4];

        /* --- Measurement noise variance (scalar, H is a single row) --- */
        float R;

        /* --- Error covariance (4x4) --- */
        float P[4][4];

        /* --- Kalman gain (last computed, for debug) --- */
        float K[4];

} KF_TypeDef;

/* =========================================================
 * API
 * ========================================================= */

extern KF_TypeDef kf;

/**
 * @brief  Generic initialisation (identity F, identity P, default Q/R).
 *         Usually called internally by KF_DCMotor_Init.
 */
void KF_Init(void);

/**
 * @brief  Set per-state process noise diagonal.
 *
 * @param q_theta    Position process noise   (small, e.g. 0.0001)
 * @param q_omega    Velocity process noise   (moderate, e.g. 0.001)
 * @param q_current  Current process noise    (moderate, e.g. 0.001)
 * @param q_load     Load-torque process noise (LARGER than others, e.g. 0.1)
 *                   Load torque is an unknown disturbance; a bigger Q lets
 *                   the filter track it without lagging.
 */
void KF_SetQ(float q_theta, float q_omega, float q_current, float q_load);

/**
 * @brief  Initialise the filter with DC motor physical parameters.
 *
 * @param dt   Sample period [s]
 * @param J    Rotor inertia [kg·m²]
 * @param Bm   Viscous friction [N·m·s/rad]
 * @param Kt   Torque constant [N·m/A]
 * @param Ke   Back-EMF constant [V·s/rad]
 * @param Rm   Armature resistance [Ω]
 * @param L    Armature inductance [H]
 * @param Q    Base process noise scalar (load gets Q*10)
 * @param R    Measurement noise variance
 */
void KF_DCMotor_Init(float dt, float J, float Bm, float Kt, float Ke, float Rm,
                     float L, float Q, float R);

/**
 * @brief  Prediction step  —  call FIRST each cycle.
 *
 *   x_prior = F x + B u
 *   P_prior = F P F' + Q
 *
 * @param voltage  Applied motor voltage [V] (control output)
 */
void KF_Predict(float voltage);

/**
 * @brief  Update step  —  call AFTER KF_Predict each cycle.
 *
 *   Uses Joseph-form update for numerical stability:
 *   P = (I-KH) P (I-KH)' + K R K'
 *
 * @param measurement  Encoder position [rad]
 */
void KF_Update(float measurement);

#endif
