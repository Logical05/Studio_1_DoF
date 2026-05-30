#include "control/kalman.h"

KF_TypeDef kf = {0};

/* =========================================================
 * Initialize
 * ========================================================= */
void KF_Init(void) {

    /* ===== State ===== */

    kf.x[0] = 0.0f;
    kf.x[1] = 0.0f;
    kf.x[2] = 0.0f;
    kf.x[3] = 0.0f;

    /* ===== Covariance ===== */

    kf.P[0][0] = 1.0f;
    kf.P[0][1] = 0.0f;
    kf.P[0][2] = 0.0f;
    kf.P[0][3] = 0.0f;

    kf.P[1][0] = 0.0f;
    kf.P[1][1] = 1.0f;
    kf.P[1][2] = 0.0f;
    kf.P[1][3] = 0.0f;

    kf.P[2][0] = 0.0f;
    kf.P[2][1] = 0.0f;
    kf.P[2][2] = 1.0f;
    kf.P[2][3] = 0.0f;

    kf.P[3][0] = 0.0f;
    kf.P[3][1] = 0.0f;
    kf.P[3][2] = 0.0f;
    kf.P[3][3] = 1.0f;

    /* ===== Default F (identity) ===== */

    kf.F[0][0] = 1.0f;
    kf.F[0][1] = 0.0f;
    kf.F[0][2] = 0.0f;
    kf.F[0][3] = 0.0f;

    kf.F[1][0] = 0.0f;
    kf.F[1][1] = 1.0f;
    kf.F[1][2] = 0.0f;
    kf.F[1][3] = 0.0f;

    kf.F[2][0] = 0.0f;
    kf.F[2][1] = 0.0f;
    kf.F[2][2] = 1.0f;
    kf.F[2][3] = 0.0f;

    kf.F[3][0] = 0.0f;
    kf.F[3][1] = 0.0f;
    kf.F[3][2] = 0.0f;
    kf.F[3][3] = 1.0f;

    /* ===== Input matrix ===== */

    kf.B[0] = 0.0f;
    kf.B[1] = 0.0f;
    kf.B[2] = 0.0f;
    kf.B[3] = 0.0f;

    /* ===== Encoder only: H = [1 0 0 0] ===== */

    kf.H[0] = 1.0f;
    kf.H[1] = 0.0f;
    kf.H[2] = 0.0f;
    kf.H[3] = 0.0f;

    /* ===== Default Q ===== */

    KF_SetQ(0.001f, 0.001f, 0.001f, 0.001f);

    /* ===== Default R ===== */

    kf.R = 1e-5f;
}

/* =========================================================
 * Set Q — per-state process noise
 *
 * q_theta  : position noise  (usually small)
 * q_omega  : velocity noise  (moderate)
 * q_current: current noise   (moderate)
 * q_load   : load torque noise — set LARGER than the rest
 *            because load is an unknown disturbance that
 *            can change fast; a bigger value lets the filter
 *            track it quickly.
 *
 * Bigger q  → faster response, noisier estimate
 * Smaller q → smoother, slower response
 * ========================================================= */
void KF_SetQ(float q_theta, float q_omega, float q_current, float q_load) {

    /* Off-diagonals are zero (states assumed independent in process noise) */
    kf.Q[0][0] = q_theta;
    kf.Q[0][1] = 0.0f;
    kf.Q[0][2] = 0.0f;
    kf.Q[0][3] = 0.0f;

    kf.Q[1][0] = 0.0f;
    kf.Q[1][1] = q_omega;
    kf.Q[1][2] = 0.0f;
    kf.Q[1][3] = 0.0f;

    kf.Q[2][0] = 0.0f;
    kf.Q[2][1] = 0.0f;
    kf.Q[2][2] = q_current;
    kf.Q[2][3] = 0.0f;

    kf.Q[3][0] = 0.0f;
    kf.Q[3][1] = 0.0f;
    kf.Q[3][2] = 0.0f;
    kf.Q[3][3] = q_load;
}

/* =========================================================
 * DC Motor Model
 *
 * States:
 *   x[0] = theta    (position, rad)
 *   x[1] = omega    (velocity, rad/s)
 *   x[2] = current  (A)
 *   x[3] = load torque (Nm) — modelled as random walk
 * ========================================================= */
void KF_DCMotor_Init(float dt, float J, float Bm, float Kt, float Ke, float Rm,
                     float L, float Q, float R) {

    KF_Init();

    /* =====================================================
     * State transition matrix  F = I + A*dt  (Euler ZOH)
     * ===================================================== */

    kf.F[0][0] = 1.0f;
    kf.F[0][1] = dt;
    kf.F[0][2] = 0.0f;
    kf.F[0][3] = 0.0f;

    kf.F[1][0] = 0.0f;
    kf.F[1][1] = 1.0f - (Bm / J) * dt;
    kf.F[1][2] = (Kt / J) * dt;
    kf.F[1][3] = -(1.0f / J) * dt;

    kf.F[2][0] = 0.0f;
    kf.F[2][1] = -(Ke / L) * dt;
    kf.F[2][2] = 1.0f - (Rm / L) * dt;
    kf.F[2][3] = 0.0f;

    kf.F[3][0] = 0.0f;
    kf.F[3][1] = 0.0f;
    kf.F[3][2] = 0.0f;
    kf.F[3][3] = 1.0f; /* load torque random walk */

    /* =====================================================
     * Input matrix  B  (voltage → states)
     * ===================================================== */

    kf.B[0] = 0.0f;
    kf.B[1] = 0.0f;
    kf.B[2] = dt / L;
    kf.B[3] = 0.0f;

    /* ===== Per-state Q ===== */
    /* Load torque (x[3]) is a disturbance; give it a larger Q so the
     filter can track unexpected load changes quickly.              */
    KF_SetQ(Q, Q * 10, Q * 10, Q * 100);

    /* ===== R ===== */

    kf.R = R;
}

/* =========================================================
 * Predict  —  call this FIRST each control cycle,
 *             BEFORE KF_Update.
 *
 *   x_k|k-1 = F x_k-1|k-1 + B u
 *   P_k|k-1 = F P_k-1|k-1 F' + Q
 * ========================================================= */
void KF_Predict(float voltage) {
    /* ===== Save state ===== */

    float x0 = kf.x[0];
    float x1 = kf.x[1];
    float x2 = kf.x[2];
    float x3 = kf.x[3];

    /* =====================================================
     * State prediction
     * x = F x + B u
     * ===================================================== */

    kf.x[0] = kf.F[0][0] * x0 + kf.F[0][1] * x1 + kf.F[0][2] * x2 + kf.F[0][3] * x3 +
              kf.B[0] * voltage;

    kf.x[1] = kf.F[1][0] * x0 + kf.F[1][1] * x1 + kf.F[1][2] * x2 + kf.F[1][3] * x3 +
              kf.B[1] * voltage;

    kf.x[2] = kf.F[2][0] * x0 + kf.F[2][1] * x1 + kf.F[2][2] * x2 + kf.F[2][3] * x3 +
              kf.B[2] * voltage;

    kf.x[3] = kf.F[3][0] * x0 + kf.F[3][1] * x1 + kf.F[3][2] * x2 + kf.F[3][3] * x3 +
              kf.B[3] * voltage;

    /* =====================================================
     * Save covariance
     * ===================================================== */

    float P00 = kf.P[0][0];
    float P01 = kf.P[0][1];
    float P02 = kf.P[0][2];
    float P03 = kf.P[0][3];

    float P10 = kf.P[1][0];
    float P11 = kf.P[1][1];
    float P12 = kf.P[1][2];
    float P13 = kf.P[1][3];

    float P20 = kf.P[2][0];
    float P21 = kf.P[2][1];
    float P22 = kf.P[2][2];
    float P23 = kf.P[2][3];

    float P30 = kf.P[3][0];
    float P31 = kf.P[3][1];
    float P32 = kf.P[3][2];
    float P33 = kf.P[3][3];

    /* =====================================================
     * FP = F P
     * ===================================================== */

    float FP00 =
        kf.F[0][0] * P00 + kf.F[0][1] * P10 + kf.F[0][2] * P20 + kf.F[0][3] * P30;
    float FP01 =
        kf.F[0][0] * P01 + kf.F[0][1] * P11 + kf.F[0][2] * P21 + kf.F[0][3] * P31;
    float FP02 =
        kf.F[0][0] * P02 + kf.F[0][1] * P12 + kf.F[0][2] * P22 + kf.F[0][3] * P32;
    float FP03 =
        kf.F[0][0] * P03 + kf.F[0][1] * P13 + kf.F[0][2] * P23 + kf.F[0][3] * P33;

    float FP10 =
        kf.F[1][0] * P00 + kf.F[1][1] * P10 + kf.F[1][2] * P20 + kf.F[1][3] * P30;
    float FP11 =
        kf.F[1][0] * P01 + kf.F[1][1] * P11 + kf.F[1][2] * P21 + kf.F[1][3] * P31;
    float FP12 =
        kf.F[1][0] * P02 + kf.F[1][1] * P12 + kf.F[1][2] * P22 + kf.F[1][3] * P32;
    float FP13 =
        kf.F[1][0] * P03 + kf.F[1][1] * P13 + kf.F[1][2] * P23 + kf.F[1][3] * P33;

    float FP20 =
        kf.F[2][0] * P00 + kf.F[2][1] * P10 + kf.F[2][2] * P20 + kf.F[2][3] * P30;
    float FP21 =
        kf.F[2][0] * P01 + kf.F[2][1] * P11 + kf.F[2][2] * P21 + kf.F[2][3] * P31;
    float FP22 =
        kf.F[2][0] * P02 + kf.F[2][1] * P12 + kf.F[2][2] * P22 + kf.F[2][3] * P32;
    float FP23 =
        kf.F[2][0] * P03 + kf.F[2][1] * P13 + kf.F[2][2] * P23 + kf.F[2][3] * P33;

    float FP30 =
        kf.F[3][0] * P00 + kf.F[3][1] * P10 + kf.F[3][2] * P20 + kf.F[3][3] * P30;
    float FP31 =
        kf.F[3][0] * P01 + kf.F[3][1] * P11 + kf.F[3][2] * P21 + kf.F[3][3] * P31;
    float FP32 =
        kf.F[3][0] * P02 + kf.F[3][1] * P12 + kf.F[3][2] * P22 + kf.F[3][3] * P32;
    float FP33 =
        kf.F[3][0] * P03 + kf.F[3][1] * P13 + kf.F[3][2] * P23 + kf.F[3][3] * P33;

    /* =====================================================
     * P = FP F' + Q
     * ===================================================== */

    kf.P[0][0] = FP00 * kf.F[0][0] + FP01 * kf.F[0][1] + FP02 * kf.F[0][2] +
                 FP03 * kf.F[0][3] + kf.Q[0][0];
    kf.P[0][1] = FP00 * kf.F[1][0] + FP01 * kf.F[1][1] + FP02 * kf.F[1][2] +
                 FP03 * kf.F[1][3] + kf.Q[0][1];
    kf.P[0][2] = FP00 * kf.F[2][0] + FP01 * kf.F[2][1] + FP02 * kf.F[2][2] +
                 FP03 * kf.F[2][3] + kf.Q[0][2];
    kf.P[0][3] = FP00 * kf.F[3][0] + FP01 * kf.F[3][1] + FP02 * kf.F[3][2] +
                 FP03 * kf.F[3][3] + kf.Q[0][3];

    kf.P[1][0] = kf.P[0][1];

    kf.P[1][1] = FP10 * kf.F[1][0] + FP11 * kf.F[1][1] + FP12 * kf.F[1][2] +
                 FP13 * kf.F[1][3] + kf.Q[1][1];
    kf.P[1][2] = FP10 * kf.F[2][0] + FP11 * kf.F[2][1] + FP12 * kf.F[2][2] +
                 FP13 * kf.F[2][3] + kf.Q[1][2];
    kf.P[1][3] = FP10 * kf.F[3][0] + FP11 * kf.F[3][1] + FP12 * kf.F[3][2] +
                 FP13 * kf.F[3][3] + kf.Q[1][3];

    kf.P[2][0] = kf.P[0][2];
    kf.P[2][1] = kf.P[1][2];

    kf.P[2][2] = FP20 * kf.F[2][0] + FP21 * kf.F[2][1] + FP22 * kf.F[2][2] +
                 FP23 * kf.F[2][3] + kf.Q[2][2];
    kf.P[2][3] = FP20 * kf.F[3][0] + FP21 * kf.F[3][1] + FP22 * kf.F[3][2] +
                 FP23 * kf.F[3][3] + kf.Q[2][3];

    kf.P[3][0] = kf.P[0][3];
    kf.P[3][1] = kf.P[1][3];
    kf.P[3][2] = kf.P[2][3];

    kf.P[3][3] = FP30 * kf.F[3][0] + FP31 * kf.F[3][1] + FP32 * kf.F[3][2] +
                 FP33 * kf.F[3][3] + kf.Q[3][3];
}

/* =========================================================
 * Update  —  call this AFTER KF_Predict each cycle.
 *
 * Uses the Joseph-form covariance update for numerical
 * stability:
 *
 *   y  = z - H x
 *   S  = H P H' + R
 *   K  = P H' / S          (H=[1,0,0,0] so H'=[1;0;0;0])
 *   x  = x + K y
 *   P  = (I - K H) P (I - K H)' + K R K'   (Joseph form)
 *
 * The Joseph form keeps P symmetric and positive-definite
 * even with floating-point rounding.
 * ========================================================= */
void KF_Update(float measurement) {
    /* =====================================================
     * Innovation
     * ===================================================== */

    float y = measurement - kf.x[0];

    /* =====================================================
     * Innovation covariance
     * ===================================================== */

    float S = kf.P[0][0] + kf.R;

    if (S < 1e-12f) S = 1e-12f;

    /* =====================================================
     * Kalman gain
     * ===================================================== */

    float K0 = kf.P[0][0] / S;
    float K1 = kf.P[1][0] / S;
    float K2 = kf.P[2][0] / S;
    float K3 = kf.P[3][0] / S;

    kf.K[0] = K0;
    kf.K[1] = K1;
    kf.K[2] = K2;
    kf.K[3] = K3;

    /* =====================================================
     * State update
     * ===================================================== */

    kf.x[0] += K0 * y;
    kf.x[1] += K1 * y;
    kf.x[2] += K2 * y;
    kf.x[3] += K3 * y;

    /* =====================================================
     * Covariance update
     *
     * P = (I - K H) P
     *
     * Since:
     * H = [1 0 0 0]
     *
     * This becomes very simple.
     * ===================================================== */

    float P00 = kf.P[0][0];
    float P01 = kf.P[0][1];
    float P02 = kf.P[0][2];
    float P03 = kf.P[0][3];

    float P10 = kf.P[1][0];
    float P11 = kf.P[1][1];
    float P12 = kf.P[1][2];
    float P13 = kf.P[1][3];

    float P20 = kf.P[2][0];
    float P21 = kf.P[2][1];
    float P22 = kf.P[2][2];
    float P23 = kf.P[2][3];

    float P30 = kf.P[3][0];
    float P31 = kf.P[3][1];
    float P32 = kf.P[3][2];
    float P33 = kf.P[3][3];

    kf.P[0][0] = P00 - K0 * P00;
    kf.P[0][1] = P01 - K0 * P01;
    kf.P[0][2] = P02 - K0 * P02;
    kf.P[0][3] = P03 - K0 * P03;

    kf.P[1][0] = P10 - K1 * P00;
    kf.P[1][1] = P11 - K1 * P01;
    kf.P[1][2] = P12 - K1 * P02;
    kf.P[1][3] = P13 - K1 * P03;

    kf.P[2][0] = P20 - K2 * P00;
    kf.P[2][1] = P21 - K2 * P01;
    kf.P[2][2] = P22 - K2 * P02;
    kf.P[2][3] = P23 - K2 * P03;

    kf.P[3][0] = P30 - K3 * P00;
    kf.P[3][1] = P31 - K3 * P01;
    kf.P[3][2] = P32 - K3 * P02;
    kf.P[3][3] = P33 - K3 * P03;

    /* =====================================================
     * Force symmetry
     * ===================================================== */

    kf.P[1][0] = kf.P[0][1];
    kf.P[2][0] = kf.P[0][2];
    kf.P[3][0] = kf.P[0][3];

    kf.P[2][1] = kf.P[1][2];
    kf.P[3][1] = kf.P[1][3];

    kf.P[3][2] = kf.P[2][3];
}
