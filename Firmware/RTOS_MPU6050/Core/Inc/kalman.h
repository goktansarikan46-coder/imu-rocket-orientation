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
    /* --- Ayar parametreleri (init'te verilir) --------------------------- */
    float Q_angle;    /* jiroskop entegrasyonunun guven aciligi            */
    float Q_bias;     /* bias'in ne hizla kaymasina izin verildigi         */
    float R_measure;  /* ivmeolcer olcumunun gurultu varyansi              */

    /* --- Durumlar ------------------------------------------------------- */
    float angle;      /* filtrenin cikisi [derece]                          */
    float bias;       /* tahmin edilen jiroskop kaymasi [dps]               */
    float rate;       /* bias cikarilmis temiz acisal hiz [dps]             */

    /* --- Hata kovaryans matrisi P (2x2) --------------------------------- */
    float P[2][2];    /* "su an ne kadar eminim" bilgisi                    */

    /* --- Davranis bayragi ----------------------------------------------- */
    uint8_t wrap180;  /* 1 ise aci -180..+180 arasinda sarilir (ROLL icin)  */
                      /* 0 ise sarma yapilmaz            (PITCH icin)       */
} Kalman_t;


/**
  * @brief  Filtreyi baslatir.
  * @param  kf         : filtre nesnesi
  * @param  start_angle: baslangic acisi (ivmeolcerden okunan deger)
  * @param  wrap180    : 1 = roll gibi sarmali eksen, 0 = pitch gibi duz eksen
  */
void kalman_init(Kalman_t *kf, float start_angle, uint8_t wrap180);


/**
  * @brief  Bir adim calistirir (predict + update).
  * @param  kf       : filtre nesnesi
  * @param  new_angle: ivmeolcerden gelen mutlak aci [derece]
  * @param  new_rate : jiroskoptan gelen acisal hiz  [dps]
  * @param  dt       : gecen sure [saniye]
  * @retval Suzulmus aci [derece]
  */
float kalman_update(Kalman_t *kf, float new_angle, float new_rate, float dt);


/**
  * @brief  Ivmeolcere duyulan guveni gecici olarak degistirir.
  *
  *  Roket havalandiginda ivmeolcer sadece yercekimini olcmez, motorun
  *  ivmesini de olcer. O anda ivmeolcerden cikan aci ANLAMSIZDIR.
  *  Bu fonksiyonla R_measure buyutulerek filtreye "su an ivmeolcere
  *  guvenme, jiroskopla devam et" denir.
  *
  * @param  kf : filtre nesnesi
  * @param  R  : yeni olcum gurultusu (normal ~0.03 , guvensiz ~10.0)
  */
void kalman_set_R(Kalman_t *kf, float R);


/* --- MPU-6050 icin sahada oturmus baslangic degerleri ------------------- */
#define KF_Q_ANGLE      0.001f
#define KF_Q_BIAS       0.003f
#define KF_R_MEASURE    0.03f

/* Ivmeolcer guvenilmez oldugunda kullanilacak "kor" gurultu degeri */
#define KF_R_BLIND     10.0f

#endif /* KALMAN_H */







