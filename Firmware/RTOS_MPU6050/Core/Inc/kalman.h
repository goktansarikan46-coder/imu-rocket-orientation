/*
 * kalman.h
 *
 *  Created on: Aug 5, 2026
 *      Author: goktan.sarikan
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

typedef struct
{
    /* --- Tuning parameters (set in init) -------------------------------- */
    float Q_angle;    /* process noise of the gyro integration              */
    float Q_bias;     /* how fast the gyro bias is allowed to drift         */
    float R_measure;  /* noise variance of the accelerometer measurement    */

    /* --- States --------------------------------------------------------- */
    float angle;      /* filter output [deg]                                */
    float bias;       /* estimated gyro bias [dps]                          */
    float rate;       /* bias-corrected angular rate [dps]                  */

    /* --- Error covariance matrix P (2x2) -------------------------------- */
    float P[2][2];    /* "how sure am I right now"                          */

    /* --- Behaviour flag ------------------------------------------------- */
    uint8_t wrap180;  /* 1: angle is wrapped to -180..+180 (for ROLL)       */
                      /* 0: no wrapping                   (for PITCH)       */
} Kalman_t;


/**
  * @brief  Initialises the filter.
  * @param  kf         : filter object
  * @param  start_angle: initial angle (from the accelerometer)
  * @param  wrap180    : 1 = wrapping axis like roll, 0 = linear axis like pitch
  */
void kalman_init(Kalman_t *kf, float start_angle, uint8_t wrap180);


/**
  * @brief  Runs one step (predict + update).
  * @param  kf       : filter object
  * @param  new_angle: absolute angle from the accelerometer [deg]
  * @param  new_rate : angular rate from the gyroscope       [dps]
  * @param  dt       : elapsed time [s]
  * @retval Filtered angle [deg]
  */
float kalman_update(Kalman_t *kf, float new_angle, float new_rate, float dt);


/**
  * @brief  Temporarily changes how much the accelerometer is trusted.
  *
  *  During powered flight the accelerometer measures thrust as well as
  *  gravity, so the angle computed from it is MEANINGLESS. Raising
  *  R_measure tells the filter "ignore the accelerometer for now and
  *  continue on the gyro".
  *
  * @param  kf : filter object
  * @param  R  : new measurement noise (normal ~0.03, untrusted ~10.0)
  */
void kalman_set_R(Kalman_t *kf, float R);


/* --- Field-tuned starting values for the MPU-6050 ----------------------- */
#define KF_Q_ANGLE      0.001f
#define KF_Q_BIAS       0.003f
#define KF_R_MEASURE    0.03f

/* "Blind" noise value used when the accelerometer cannot be trusted */
#define KF_R_BLIND     10.0f

#endif /* KALMAN_H */
