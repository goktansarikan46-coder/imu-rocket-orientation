/*
 * kalman.c
 *
 *  Created on: Aug 5, 2026
 *      Author: goktan.sarikan
 */

#include "kalman.h"


/* ==========================================================================
 *  Yardimci: aciyi -180..+180 araligina sarar
 * ========================================================================== */
static float wrap_180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}


/* ==========================================================================
 *  BASLATMA
 * ========================================================================== */
void kalman_init(Kalman_t *kf, float start_angle, uint8_t wrap180)
{
    kf->Q_angle   = KF_Q_ANGLE;
    kf->Q_bias    = KF_Q_BIAS;
    kf->R_measure = KF_R_MEASURE;

    kf->angle = start_angle;   /* ivmeolcerden okunan ilk aci */
    kf->bias  = 0.0f;          /* bias'i bilmiyoruz, sifirdan basla */
    kf->rate  = 0.0f;

    /* P matrisi: baslangicta aciya guveniyoruz (ivmeolcerden geldi),
       bias'i ise hic bilmiyoruz -> P[1][1] biraz buyuk baslar.        */
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.1f;

    kf->wrap180 = wrap180;
}


/* ==========================================================================
 *  BIR ADIM: PREDICT + UPDATE
 * ========================================================================== */
float kalman_update(Kalman_t *kf, float new_angle, float new_rate, float dt)
{
    /* ----------------------------------------------------------------------
     *  1) PREDICT  -  jiroskopla ilerle, belirsizligi BUYUT
     * ------------------------------------------------------------------- */

    /* Jiroskop hizindan tahmini bias'i cikar -> temiz hiz */
    kf->rate   = new_rate - kf->bias;

    /* Aciyi integre et:  aci = aci + hiz * dt
       (Tamamlayici filtredeki  r_pred = roll + fgx*dt  ile ayni satir)  */
    kf->angle += kf->rate * dt;

    /* Belirsizlik yayilimi:  P = A*P*A' + Q
       Matris carpimi acilmis hali:                                       */
    kf->P[0][0] += dt * ( dt * kf->P[1][1]
                          - kf->P[0][1]
                          - kf->P[1][0]
                          + kf->Q_angle );
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;


    /* ----------------------------------------------------------------------
     *  2) UPDATE  -  ivmeolcer olcumuyle duzelt, belirsizligi KUCULT
     * ------------------------------------------------------------------- */

    /* Yenilik (innovation): olcum ile tahmin arasindaki fark.
       Tamamlayici filtredeki  err = r_acc - r_pred  ile ayni sey.        */
    float y = new_angle - kf->angle;

    /* Roll ekseninde -180/+180 sinirinda basa sarma duzeltmesi.
       Ornek: olcum +179, tahmin -179 ise fark 358 degil, -2 derecedir.   */
    if (kf->wrap180) y = wrap_180(y);

    /* Yenilik kovaryansi: S = H*P*H' + R  =  P[0][0] + R                 */
    float S = kf->P[0][0] + kf->R_measure;

    /* KALMAN KAZANCI:  K = P*H' / S
       K0 -> aciya uygulanacak duzeltme orani  (senin ALPHA'nin yerine gecen sayi)
       K1 -> bias'a uygulanacak duzeltme orani                            */
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    /* Durumlari duzelt:  x = x + K*y                                     */
    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    /* Kovaryansi guncelle:  P = (I - K*H) * P
       Eski degerler lazim oldugu icin once kopyala.                      */
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];

    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] -= K1 * P00;
    kf->P[1][1] -= K1 * P01;

    /* Cikisi araliga otur (sadece roll icin) */
    if (kf->wrap180) kf->angle = wrap_180(kf->angle);

    return kf->angle;
}


/* ==========================================================================
 *  IVMEOLCER GUVENINI DEGISTIR
 * ========================================================================== */
void kalman_set_R(Kalman_t *kf, float R)
{
    kf->R_measure = R;
}
