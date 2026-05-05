/*
 * kalman.h
 *
 *  Created on: Apr 8, 2026
 *      Author: rajap
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

typedef struct {
	float x[2];        // state
	float P[2][2];     // covariance
	float F[2][2];     // state transition
	float H[2];        // measurement matrix
	float Q[2][2];     // process noise
	float R;           // measurement noise
	float K[2];        // kalman gain
} KF_TypeDef;

void KF_Init(KF_TypeDef*);
void KF_Predict(KF_TypeDef*);
void KF_Update(KF_TypeDef*, float);
void KF_SetQ_Continuous(KF_TypeDef*, float, float);
void KF_SetQ_Discrete(KF_TypeDef*, float, float);

#endif /* INC_KALMAN_H_ */
