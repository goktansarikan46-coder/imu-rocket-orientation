/*
 * kalman.c
 *
 *  Created on: Aug 5, 2026
 *      Author: goktan.sarikan
 */

#include "kalman.h"


/* ==========================================================================
 *  Helper: wraps an angle to the -180..+180 range
 * ========================================================================== */
static float wrap_180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}


/* ==========================================================================
 *  INIT
 * ========================================================================== */
void kalman_init(Kalman_t *kf, float start_angle, uint8_t wrap180)
{
    kf->Q_angle   = KF_Q_ANGLE;
    kf->Q_bias    = KF_Q_BIAS;
    kf->R_measure = KF_R_MEASURE;

    kf->angle = start_angle;   /* first angle read from the accelerometer */
    kf->bias  = 0.0f;          /* bias unknown, start from zero */
    kf->rate  = 0.0f;

    /* P matrix: the initial angle is trusted (it came from the accelerometer),
       the bias is unknown -> P[1][1] starts somewhat large.               */
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.1f;

    kf->wrap180 = wrap180;
}


/* ==========================================================================
 *  ONE STEP: PREDICT + UPDATE
 * ========================================================================== */
float kalman_update(Kalman_t *kf, float new_angle, float new_rate, float dt)
{
    /* ----------------------------------------------------------------------
     *  1) PREDICT  -  propagate with the gyro, GROW the uncertainty
     * ------------------------------------------------------------------- */

    /* Subtract the estimated bias from the gyro rate -> clean rate */
    kf->rate   = new_rate - kf->bias;

    /* Integrate the angle:  angle = angle + rate * dt
       (same line as  r_pred = roll + fgx*dt  in the complementary filter) */
    kf->angle += kf->rate * dt;

    /* Covariance propagation:  P = A*P*A' + Q
       Matrix product written out:                                        */
    kf->P[0][0] += dt * ( dt * kf->P[1][1]
                          - kf->P[0][1]
                          - kf->P[1][0]
                          + kf->Q_angle );
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;


    /* ----------------------------------------------------------------------
     *  2) UPDATE  -  correct with the accelerometer, SHRINK the uncertainty
     * ------------------------------------------------------------------- */

    /* Innovation: difference between measurement and prediction.
       Same as  err = r_acc - r_pred  in the complementary filter.        */
    float y = new_angle - kf->angle;

    /* Wrap-around fix at the -180/+180 boundary on the roll axis.
       Example: measurement +179, prediction -179 -> the error is -2 deg, not 358. */
    if (kf->wrap180) y = wrap_180(y);

    /* Innovation covariance: S = H*P*H' + R  =  P[0][0] + R              */
    float S = kf->P[0][0] + kf->R_measure;

    /* KALMAN GAIN:  K = P*H' / S
       K0 -> correction ratio applied to the angle  (replaces the complementary ALPHA)
       K1 -> correction ratio applied to the bias                         */
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    /* Correct the states:  x = x + K*y                                   */
    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    /* Update the covariance:  P = (I - K*H) * P
       The old values are needed, so copy them first.                     */
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];

    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;

    /* Keep the output in range (roll only) */
    if (kf->wrap180) kf->angle = wrap_180(kf->angle);

    return kf->angle;
}


/* ==========================================================================
 *  CHANGE ACCELEROMETER TRUST
 * ========================================================================== */
void kalman_set_R(Kalman_t *kf, float R)
{
    kf->R_measure = R;
}
