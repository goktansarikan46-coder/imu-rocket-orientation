/******************************************************************************
  * @file    app_threadx.c
  * @brief   MPU-6050 + Kalman Filter + ThreadX
  *          STM32C092RC (Cortex-M0+)
  *          Goktan Sarikan
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "kalman.h"

/* ============================================================================
 *  AYARLAR
 * ==========================================================================*/

/* --- MPU-6050 --- */
#define MPU_ADDR        (0x68 << 1)   /* HAL 8-bit adres ister      */
#define MPU_REG_DATA    0x3B          /* ACC_XOUT_H (14 byte blok)  */
#define MPU_REG_WHOAMI  0x75

/* --- Olcek katsayilari --- */
#define GYRO_LSB        65.5f         /* +-500 dps -> LSB/(derece/s) */
#define RAD2DEG         57.29578f

/* --- Ivmeolcer kalibrasyon katsayilari (6-pozisyon testi sonucu) ----------
 *    offset = (pozitif_olcum + negatif_olcum) / 2   [LSB]
 *    scale  = (pozitif_olcum - negatif_olcum) / 2   [LSB/g]
 * -------------------------------------------------------------------------*/
#define ACC_OFF_X       651.5f
#define ACC_OFF_Y       415.5f
#define ACC_OFF_Z       800.5f

#define ACC_SCL_X       16359.5f
#define ACC_SCL_Y       16393.5f
#define ACC_SCL_Z       16863.5f

/* --- Olcume guven esikleri ----------------------------------------------- */
#define ACC_TRUST_BAND  0.15f    /* |a| bundan fazla saparsa hareket var    */
#define ROLL_BLIND_GYZ  0.15f    /* yatay yercekimi payi -> roll gorunmez   */

/* --- Zamanlama --- */
#define SAMPLE_TICKS    1        /* 1 ThreadX tick = 10 ms -> 100 Hz        */
#define GYRO_CAL_SAMPLES 1000    /* 1000 x 10 ms = 10 saniye                */

/* ms -> tick cevirici (tick frekansi degisirse kod bozulmasin diye) */
#define MS_TO_TICK(ms)  (((ms) * TX_TIMER_TICKS_PER_SECOND) / 1000)

/* --- Thread ayarlari --- */
#define IMU_STACK_SIZE   1024
#define UART_STACK_SIZE  1024
#define IMU_PRIORITY     5       /* kucuk sayi = yuksek oncelik             */
#define UART_PRIORITY    10
#define QUEUE_DEPTH      8

/* Private typedef -----------------------------------------------------------*/

/* Sensorden okunan ham degerler (12 byte) */
typedef struct
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} imu_raw_t;

/* Kuyrukta tasinan mesaj: 8 x int16 = 16 byte = 4 ULONG (TX_4_ULONG) */
typedef struct
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t roll100;              /* derece x 100 */
  int16_t pitch100;             /* derece x 100 */
} imu_msg_t;

/* Private variables ---------------------------------------------------------*/

extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart2;

/* ThreadX nesneleri */
static TX_THREAD    imu_thread;       /* sensor okuma + filtre        */
static TX_THREAD    uart_thread;      /* CSV gonderme                 */
static TX_QUEUE     imu_queue;        /* IMU -> UART veri yolu        */
static TX_TIMER     sample_timer;     /* 100 Hz periyot uretici       */
static TX_SEMAPHORE sample_sem;       /* "yeni ornek zamani" isareti  */
static TX_SEMAPHORE uart_tx_done;     /* "UART gonderimi bitti"       */

/* Kalman */
static Kalman_t kf_roll;
static Kalman_t kf_pitch;

/* --- LIVE EXPRESSIONS izleme degiskenleri -------------------------------- */
volatile float    dbg_roll      = 0.0f;   /* filtre cikisi - roll        */
volatile float    dbg_pitch     = 0.0f;   /* filtre cikisi - pitch       */
volatile float    dbg_roll_acc  = 0.0f;   /* ham ivmeolcer acisi - roll  */
volatile float    dbg_pitch_acc = 0.0f;   /* ham ivmeolcer acisi - pitch */
volatile float    dbg_gyro_x_dps = 0.0f;  /* bias duzeltilmis jiro       */
volatile float    dbg_gyro_y_dps = 0.0f;
volatile float    dbg_bias_x    = 0.0f;   /* Kalman'in ogrendigi bias    */
volatile float    dbg_kgain     = 0.0f;   /* anlik Kalman kazanci        */
volatile float    dbg_a_mag     = 0.0f;   /* |a| : sabitken 1.000 olmali */
volatile float    dbg_g_yz      = 0.0f;   /* roll gozlenebilirligi       */
volatile uint8_t  dbg_who       = 0;      /* WHO_AM_I (104 olmali)       */
volatile uint32_t dbg_sample_cnt = 0;     /* ornek sayaci                */

/* Private function prototypes -----------------------------------------------*/
static void imu_thread_entry(ULONG arg);
static void uart_thread_entry(ULONG arg);
static void sample_timer_cb(ULONG arg);

static void mpu_write(uint8_t reg, uint8_t val);
static void mpu_read(imu_raw_t *r);
static void mpu_init_regs(void);
static void uart_send_blocking(const char *s);

/* ============================================================================
 *  1. BOLUM : MPU-6050 SURUCUSU
 * ==========================================================================*/

/* Tek bir registera tek byte yaz */
static void mpu_write(uint8_t reg, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/* 0x3B'den 14 byte oku: ACC X/Y/Z, TEMP, GYRO X/Y/Z (hepsi big-endian) */
static void mpu_read(imu_raw_t *r)
{
  uint8_t b[14];

  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_REG_DATA, I2C_MEMADD_SIZE_8BIT, b, 14, 100);

  r->ax = (int16_t)(b[0]  << 8 | b[1]);
  r->ay = (int16_t)(b[2]  << 8 | b[3]);
  r->az = (int16_t)(b[4]  << 8 | b[5]);
  r->gx = (int16_t)(b[8]  << 8 | b[9]);
  r->gy = (int16_t)(b[10] << 8 | b[11]);
  r->gz = (int16_t)(b[12] << 8 | b[13]);
}

/* Olcek ve filtre ayarlari */
static void mpu_init_regs(void)
{
  mpu_write(0x1A, 0x03);   /* DLPF 42 Hz  */
  mpu_write(0x19, 0x09);   /* 100 Hz      */
  mpu_write(0x1B, 0x08);   /* gyro +-500  */
  mpu_write(0x1C, 0x00);   /* acc  +-2 g  */
}


static void uart_send_blocking(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 100);
}

/* ============================================================================
 *  2. BOLUM : ZAMANLAYICI  -  100 Hz periyodu uretir
 *
 *  Her ThreadX tick'inde (10 ms) calisir ve sadece semaforu bir arttirir.
 *  Tavan (ceiling) 1 verildigi icin IMU_Thread gecikirse semafor birikmez;
 *  thread "yetismek icin" pes pese kosmaya calismaz, temiz baslar.
 * ==========================================================================*/
static void sample_timer_cb(ULONG arg)
{
  (void)arg;
  tx_semaphore_ceiling_put(&sample_sem, 1);
}

/* ============================================================================
 *  3. BOLUM : IMU THREAD  -  oku, filtrele, kuyruga koy
 * ==========================================================================*/
static void imu_thread_entry(ULONG arg)
{
  imu_raw_t raw;
  imu_msg_t msg;
  char      banner[64];
  uint8_t   who = 0;

  float roll = 0.0f, pitch = 0.0f;   /* filtre ciktisi [derece]  */
  float bx   = 0.0f, by    = 0.0f;   /* jiroskop bias [LSB]      */

  /* dt SABIT: timer periyodundan hesaplanir, olcumden degil.
     Degisken dt = integrasyon hatasina sebep olur. Önceki kodumda hatalı olan kısım. */
  const float dt = (float)SAMPLE_TICKS / (float)TX_TIMER_TICKS_PER_SECOND;

  (void)arg;

  /* ---- 3.1  MPU-6050'yi baslat --------------------------------------- */
  mpu_write(0x6B, 0x80);  tx_thread_sleep(MS_TO_TICK(100));  /* reset        */
  mpu_write(0x6B, 0x01);  tx_thread_sleep(MS_TO_TICK(50));   /* uyandir      */
  mpu_init_regs();                                           /* olcek/filtre */
  tx_thread_sleep(MS_TO_TICK(50));

  /* ---- 3.2  Kimlik kontrolu ------------------------------------------ */
  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_REG_WHOAMI, I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
  dbg_who = who;
  sprintf(banner, "WHO_AM_I=%d (104 olmali)\r\n", who);
  uart_send_blocking(banner);

  /* ---- 3.3  Periyot uretecini baslat --------------------------------- */
  tx_timer_activate(&sample_timer);

  /* ---- 3.4  Jiroskop bias kalibrasyonu -------------------------------
     Bu HER ACILISTA gereklidir: jiro offset'i sicaklikla ve zamanla
     degisir, sabit sayiya yazilamaz. Kart bu sure boyunca SABIT durmali. */
  {
    long sx = 0, sy = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
      tx_semaphore_get(&sample_sem, TX_WAIT_FOREVER);   /* 10 ms bekle */
      mpu_read(&raw);
      sx += raw.gx;
      sy += raw.gy;
    }
    bx = (float)sx / (float)GYRO_CAL_SAMPLES;
    by = (float)sy / (float)GYRO_CAL_SAMPLES;
  }

  /* ---- 3.5  Baslangic acilari (sadece ivmeolcerden) ------------------
     Kalibrasyon duzeltmesi burada da uygulanir; yoksa filtre yanlis bir
     baslangic noktasindan yola cikar. */
  mpu_read(&raw);
  {
    float fax0 = (raw.ax - ACC_OFF_X) / ACC_SCL_X;
    float fay0 = (raw.ay - ACC_OFF_Y) / ACC_SCL_Y;
    float faz0 = (raw.az - ACC_OFF_Z) / ACC_SCL_Z;

    roll  = atan2f(fay0, faz0) * RAD2DEG;
    pitch = atan2f(-fax0, sqrtf(fay0 * fay0 + faz0 * faz0)) * RAD2DEG;
  }

  kalman_init(&kf_roll,  roll,  1);
  kalman_init(&kf_pitch, pitch, 0);

  uart_send_blocking("ax,ay,az,gx,gy,gz,roll100,pitch100\r\n");

  /* ---- 3.6  ANA DONGU : 100 Hz --------------------------------------- */
  while (1)
  {
    /* Periyodun basini bekle. Thread burada CPU tuketmez. */
    tx_semaphore_get(&sample_sem, TX_WAIT_FOREVER);

    mpu_read(&raw);

    /* --- Ham degerleri fiziksel birime cevir ------------------------- */
    float fgx = (raw.gx - bx) / GYRO_LSB;            /* dps */
    float fgy = (raw.gy - by) / GYRO_LSB;

    /* Ivmeolcer: once offset cikar, sonra kendi olcegine bol. */
    float fax = (raw.ax - ACC_OFF_X) / ACC_SCL_X;    /* g */
    float fay = (raw.ay - ACC_OFF_Y) / ACC_SCL_Y;
    float faz = (raw.az - ACC_OFF_Z) / ACC_SCL_Z;

    /* Ivmeolcerden aci */
    float r_acc = atan2f(fay, faz) * RAD2DEG;
    float p_acc = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * RAD2DEG;

    /* --- OLCUME GUVEN (adaptif R) -----------------------------------
       Kalman'a "bu olcume ne kadar guveneyim" bilgisini veriyoruz.
       R buyuk -> olcumu dikkate alma, jiroya guven.                   */

    /* |a| : sadece yercekimi varken 1.000 olmali. Sapma = hareket. */
    float a_mag = sqrtf(fax * fax + fay * fay + faz * faz);

    /* Roll'un gorulebilirligi: yatay duzlemdeki yercekimi payi.
       pitch=0 -> 1.0 (mukemmel),  pitch=90 -> 0.0 (bilgi yok) */
    float g_yz  = sqrtf(fay * fay + faz * faz);

    float R_roll, R_pitch;

    if (fabsf(a_mag - 1.0f) > ACC_TRUST_BAND)
    {
      R_roll  = KF_R_BLIND;              /* hareket var, ikisine de guvenme */
      R_pitch = KF_R_BLIND;
    }
    else
    {
      R_pitch = KF_R_MEASURE;            /* pitch her acida gozlenebilir */

      if (g_yz < ROLL_BLIND_GYZ)
        R_roll = KF_R_BLIND;             /* dik duruma yakin: roll okunamaz */
      else
        R_roll = KF_R_MEASURE / (g_yz * g_yz);
    }

    kalman_set_R(&kf_roll,  R_roll);
    kalman_set_R(&kf_pitch, R_pitch);

    /* --- KALMAN FILTRE ----------------------------------------------- */
    roll  = kalman_update(&kf_roll,  r_acc, fgx, dt);
    pitch = kalman_update(&kf_pitch, p_acc, fgy, dt);

    /* --- Debug degiskenleri (Live Expressions) ----------------------- */
    dbg_roll       = roll;
    dbg_pitch      = pitch;
    dbg_roll_acc   = r_acc;
    dbg_pitch_acc  = p_acc;
    dbg_gyro_x_dps = fgx;
    dbg_gyro_y_dps = fgy;
    dbg_a_mag      = a_mag;
    dbg_g_yz       = g_yz;
    dbg_bias_x     = kf_roll.bias;
    dbg_kgain      = kf_roll.P[0][0] / (kf_roll.P[0][0] + kf_roll.R_measure);
    dbg_sample_cnt++;

    /* --- Sonucu kuyruga birak ---------------------------------------- */
    msg.ax       = raw.ax;
    msg.ay       = raw.ay;
    msg.az       = raw.az;
    msg.gx       = raw.gx;
    msg.gy       = raw.gy;
    msg.gz       = raw.gz;
    msg.roll100  = (int16_t)(roll  * 100.0f);
    msg.pitch100 = (int16_t)(pitch * 100.0f);

    /* TX_NO_WAIT: kuyruk dolduysa (UART yetisemiyorsa) bu ornegi at.
       Orneklemeyi ASLA bekletme - filtre zamanlamasi bozulmasin. */
    tx_queue_send(&imu_queue, &msg, TX_NO_WAIT);
  }
}

/* ============================================================================
 *  4. BOLUM : UART THREAD  -  CSV satirini gonder
 * ==========================================================================*/
static void uart_thread_entry(ULONG arg)
{
  imu_msg_t msg;
  char      line[80];
  int       len;

  (void)arg;

  while (1)
  {
    /* Kuyrukta mesaj yoksa burada uyur, CPU harcamaz. */
    tx_queue_receive(&imu_queue, &msg, TX_WAIT_FOREVER);

    len = sprintf(line, "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                  msg.ax, msg.ay, msg.az,
                  msg.gx, msg.gy, msg.gz,
                  msg.roll100, msg.pitch100);

    /* Interrupt ile gonder: CPU bosa donmez.
       'line' thread'in yiginindadir; asagida semaforu bekledigimiz icin
       gonderim bitene kadar gecerli kalir. */
    if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)line, len) == HAL_OK)
    {
      tx_semaphore_get(&uart_tx_done, MS_TO_TICK(200));   /* takilirsa kurtul */
    }
  }
}

/* Gonderim bitince USART2 kesmesinden cagrilir -> UART_Thread'i uyandirir */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    tx_semaphore_put(&uart_tx_done);
  }
}

/* ============================================================================
 *  5. BOLUM : KURULUM  -  tum ThreadX nesneleri burada olusturulur
 * ==========================================================================*/
/**
  * @brief  ThreadX uygulama nesnelerini olusturur.
  * @param  memory_ptr: CubeMX'in olusturdugu byte pool
  * @retval TX_SUCCESS veya hata kodu
  * @note   Bu fonksiyon zamanlayici (scheduler) baslamadan once calisir.
  *         Icinde HAL cagrisi YAPILMAZ (interrupt kapali).
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL *)memory_ptr;
  CHAR *ptr;

  /* --- 5.1  Kuyruk : IMU -> UART ------------------------------------- */
  if (tx_byte_allocate(byte_pool, (VOID **)&ptr,
                       QUEUE_DEPTH * sizeof(imu_msg_t), TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_queue_create(&imu_queue, "IMU Queue", TX_4_ULONG,
                      ptr, QUEUE_DEPTH * sizeof(imu_msg_t)) != TX_SUCCESS)
  {
    return TX_QUEUE_ERROR;
  }

  /* --- 5.2  Semaforlar ------------------------------------------------ */
  /* Baslangic degeri 0: iki taraf da once "olay" bekler. */
  if (tx_semaphore_create(&sample_sem, "Sample Tick", 0) != TX_SUCCESS)
  {
    return TX_SEMAPHORE_ERROR;
  }
  if (tx_semaphore_create(&uart_tx_done, "UART Done", 0) != TX_SUCCESS)
  {
    return TX_SEMAPHORE_ERROR;
  }

  /* --- 5.3  Periyot timer'i (10 ms) ----------------------------------- */
  /* TX_NO_ACTIVATE: IMU_Thread sensoru kurduktan sonra kendisi baslatir. */
  if (tx_timer_create(&sample_timer, "Sample Timer", sample_timer_cb, 0,
                      SAMPLE_TICKS, SAMPLE_TICKS, TX_NO_ACTIVATE) != TX_SUCCESS)
  {
    return TX_TIMER_ERROR;
  }

  /* --- 5.4  IMU Thread (yuksek oncelik) ------------------------------- */
  if (tx_byte_allocate(byte_pool, (VOID **)&ptr,
                       IMU_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&imu_thread, "IMU Thread", imu_thread_entry, 0,
                       ptr, IMU_STACK_SIZE,
                       IMU_PRIORITY, IMU_PRIORITY,
                       TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  /* --- 5.5  UART Thread (dusuk oncelik) ------------------------------- */
  if (tx_byte_allocate(byte_pool, (VOID **)&ptr,
                       UART_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&uart_thread, "UART Thread", uart_thread_entry, 0,
                       ptr, UART_STACK_SIZE,
                       UART_PRIORITY, UART_PRIORITY,
                       TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  return TX_SUCCESS;
}

/* ============================================================================
 *  6. BOLUM : CEKIRDEK GIRISI
 * ==========================================================================*/
void MX_ThreadX_Init(void)
{
  tx_kernel_enter();   /* geri donmez */
}
