/*
 * kalman.c
 *
 *  Created on: Apr 8, 2026
 *      Author: rajap
 */

#include <kalman.h>

void KF_Init(KF_TypeDef *kf) {
	// State
	kf->x[0] = 0.0f;
	kf->x[1] = 0.0f;

	// P (initial covariance)
	kf->P[0][0] = 1.0f;
	kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f;
	kf->P[1][1] = 1.0f;

	// default F = I
	kf->F[0][0] = 1.0f;
	kf->F[0][1] = 0.0f;
	kf->F[1][0] = 0.0f;
	kf->F[1][1] = 1.0f;

	// default H = I
	kf->H[0] = 1.0f;
	kf->H[1] = 0.0f;

	// Q
	kf->Q[0][0] = 0.001f;
	kf->Q[0][1] = 0.001f;
	kf->Q[1][0] = 0.001f;
	kf->Q[1][1] = 0.001f;

	// R
	kf->R = 0.001f;
}

void KF_Predict(KF_TypeDef *kf) {
	// x = F x
	float x0 = kf->x[0];
	float x1 = kf->x[1];

	kf->x[0] = kf->F[0][0] * x0 + kf->F[0][1] * x1;
	kf->x[1] = kf->F[1][0] * x0 + kf->F[1][1] * x1;

	// P = F P F' + Q
	float P00 = kf->P[0][0], P10 = kf->P[1][0];
	float P01 = kf->P[0][1], P11 = kf->P[1][1];

	float F00 = kf->F[0][0], F01 = kf->F[0][1];
	float F10 = kf->F[1][0], F11 = kf->F[1][1];

	// FP = F * P
	float FP00 = F00 * P00 + F01 * P10;
	float FP01 = F00 * P01 + F01 * P11;
	float FP10 = F10 * P00 + F11 * P10;
	float FP11 = F10 * P01 + F11 * P11;

	// P = FP * F' + Q
	kf->P[0][0] = FP00 * F00 + FP01 * F01 + kf->Q[0][0];
	kf->P[0][1] = FP00 * F10 + FP01 * F11 + kf->Q[0][1];
	kf->P[1][0] = kf->P[0][1];
	kf->P[1][1] = FP10 * F10 + FP11 * F11 + kf->Q[1][1];
}

void KF_Update(KF_TypeDef *kf, float z) {
	float H0 = kf->H[0];
	float H1 = kf->H[1];

	float x0 = kf->x[0];
	float x1 = kf->x[1];

	// y = z - Hx
	float y = z - (H0 * x0 + H1 * x1);

	float P00 = kf->P[0][0], P10 = kf->P[1][0];
	float P01 = kf->P[0][1], P11 = kf->P[1][1];

	// S = HPH' + R
	float S = H0 * (P00 * H0 + P01 * H1) + H1 * (P10 * H0 + P11 * H1) + kf->R;

	// K = P H' / S
	float K0 = (P00 * H0 + P01 * H1) / S;
	float K1 = (P10 * H0 + P11 * H1) / S;

	kf->K[0] = K0;
	kf->K[1] = K1;

	// x = x + K y
	kf->x[0] += K0 * y;
	kf->x[1] += K1 * y;

	// P = (I - KH)P
	kf->P[0][0] = P00 - K0 * (H0 * P00 + H1 * P10);
	kf->P[0][1] = P01 - K0 * (H0 * P01 + H1 * P11);
	kf->P[1][0] = P10 - K1 * (H0 * P00 + H1 * P10);
	kf->P[1][1] = P11 - K1 * (H0 * P01 + H1 * P11);
}

void KF_SetQ_Continuous(KF_TypeDef *kf, float sigma_a, float dt) {		//pg. 166
	float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt2 * dt2;
	float dt5 = dt4 * dt;

	float q = sigma_a * sigma_a;

	kf->Q[0][0] = dt5 * (1.0f / 20.0f) * q;
	kf->Q[0][1] = dt4 * (1.0f / 8.0f) * q;
	kf->Q[1][0] = kf->Q[0][1];
	kf->Q[1][1] = dt3 * (1.0f / 3.0f) * q;
}

void KF_SetQ_Discrete(KF_TypeDef *kf, float sigma_a, float dt) {		//pg. 163
	float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt2 * dt2;

	float q = sigma_a * sigma_a;

	kf->Q[0][0] = 0.25f * dt4 * q;
	kf->Q[0][1] = 0.5f * dt3 * q;
	kf->Q[1][0] = kf->Q[0][1];
	kf->Q[1][1] = dt2 * q;
}
