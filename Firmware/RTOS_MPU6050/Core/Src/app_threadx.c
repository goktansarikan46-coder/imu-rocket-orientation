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
 *  SETTINGS
 * ==========================================================================*/

/* --- MPU-6050 --- */
#define MPU_ADDR        (0x68 << 1)   /* HAL expects the 8-bit address */
#define MPU_REG_DATA    0x3B          /* ACC_XOUT_H (14-byte block)    */
#define MPU_REG_WHOAMI  0x75

/* --- Scale factors --- */
#define GYRO_LSB        65.5f         /* +-500 dps -> LSB/(deg/s)       */
#define RAD2DEG         57.29578f

/* --- Accelerometer calibration constants (6-position test) ----------------
 *    offset = (positive_reading + negative_reading) / 2   [LSB]
 *    scale  = (positive_reading - negative_reading) / 2   [LSB/g]
 * -------------------------------------------------------------------------*/
#define ACC_OFF_X       651.5f
#define ACC_OFF_Y       415.5f
#define ACC_OFF_Z       800.5f

#define ACC_SCL_X       16359.5f
#define ACC_SCL_Y       16393.5f
#define ACC_SCL_Z       16863.5f

/* --- Measurement trust thresholds ---------------------------------------- */
#define ACC_TRUST_BAND  0.15f    /* |a| deviates more than this -> motion   */
#define ROLL_BLIND_GYZ  0.15f    /* gravity share in Y-Z plane -> roll lost */

/* --- Timing --- */
#define SAMPLE_TICKS    1        /* 1 ThreadX tick = 10 ms -> 100 Hz        */
#define GYRO_CAL_SAMPLES 1000    /* 1000 x 10 ms = 10 seconds               */

/* ms -> tick converter (so the code still works if the tick rate changes) */
#define MS_TO_TICK(ms)  (((ms) * TX_TIMER_TICKS_PER_SECOND) / 1000)

/* --- Thread settings --- */
#define IMU_STACK_SIZE   1024
#define UART_STACK_SIZE  1024
#define IMU_PRIORITY     5       /* lower number = higher priority          */
#define UART_PRIORITY    10
#define QUEUE_DEPTH      8

/* Private typedef -----------------------------------------------------------*/

/* Raw values read from the sensor (12 bytes) */
typedef struct
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} imu_raw_t;

/* Message carried in the queue: 8 x int16 = 16 bytes = 4 ULONG (TX_4_ULONG) */
typedef struct
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t roll100;              /* degrees x 100 */
  int16_t pitch100;             /* degrees x 100 */
} imu_msg_t;

/* Private variables ---------------------------------------------------------*/

extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart2;

/* ThreadX objects */
static TX_THREAD    imu_thread;       /* sensor read + filter         */
static TX_THREAD    uart_thread;      /* CSV output                   */
static TX_QUEUE     imu_queue;        /* IMU -> UART data path        */
static TX_TIMER     sample_timer;     /* 100 Hz period generator      */
static TX_SEMAPHORE sample_sem;       /* "time for a new sample"      */
static TX_SEMAPHORE uart_tx_done;     /* "UART transmit finished"     */

/* Kalman */
static Kalman_t kf_roll;
static Kalman_t kf_pitch;

/* --- LIVE EXPRESSIONS debug variables ------------------------------------ */
volatile float    dbg_roll      = 0.0f;   /* filter output - roll        */
volatile float    dbg_pitch     = 0.0f;   /* filter output - pitch       */
volatile float    dbg_roll_acc  = 0.0f;   /* raw accel angle - roll      */
volatile float    dbg_pitch_acc = 0.0f;   /* raw accel angle - pitch     */
volatile float    dbg_gyro_x_dps = 0.0f;  /* bias-corrected gyro         */
volatile float    dbg_gyro_y_dps = 0.0f;
volatile float    dbg_bias_x    = 0.0f;   /* bias learned by the Kalman  */
volatile float    dbg_kgain     = 0.0f;   /* current Kalman gain         */
volatile float    dbg_a_mag     = 0.0f;   /* |a| : 1.000 when at rest    */
volatile float    dbg_g_yz      = 0.0f;   /* roll observability          */
volatile uint8_t  dbg_who       = 0;      /* WHO_AM_I (should be 104)    */
volatile uint32_t dbg_sample_cnt = 0;     /* sample counter              */

/* Private function prototypes -----------------------------------------------*/
static void imu_thread_entry(ULONG arg);
static void uart_thread_entry(ULONG arg);
static void sample_timer_cb(ULONG arg);

static void mpu_write(uint8_t reg, uint8_t val);
static void mpu_read(imu_raw_t *r);
static void mpu_init_regs(void);
static void uart_send_blocking(const char *s);

/* ============================================================================
 *  PART 1 : MPU-6050 DRIVER
 * ==========================================================================*/

/* Write one byte to one register */
static void mpu_write(uint8_t reg, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

/* Read 14 bytes from 0x3B: ACC X/Y/Z, TEMP, GYRO X/Y/Z (all big-endian) */
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

/* Range and filter settings */
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
 *  PART 2 : TIMER  -  generates the 100 Hz period
 *
 *  Runs on every ThreadX tick (10 ms) and only increments the semaphore.
 *  With a ceiling of 1 the semaphore cannot pile up if IMU_Thread is late,
 *  so the thread never runs back to back "to catch up"; it starts clean.
 * ==========================================================================*/
static void sample_timer_cb(ULONG arg)
{
  (void)arg;
  tx_semaphore_ceiling_put(&sample_sem, 1);
}

/* ============================================================================
 *  PART 3 : IMU THREAD  -  read, filter, put into the queue
 * ==========================================================================*/
static void imu_thread_entry(ULONG arg)
{
  imu_raw_t raw;
  imu_msg_t msg;
  char      banner[64];
  uint8_t   who = 0;

  float roll = 0.0f, pitch = 0.0f;   /* filter output [deg]      */
  float bx   = 0.0f, by    = 0.0f;   /* gyro bias [LSB]          */

  /* dt is FIXED: derived from the timer period, not measured.
     A variable dt causes integration error. This was the bug in my earlier code. */
  const float dt = (float)SAMPLE_TICKS / (float)TX_TIMER_TICKS_PER_SECOND;

  (void)arg;

  /* ---- 3.1  Start the MPU-6050 --------------------------------------- */
  mpu_write(0x6B, 0x80);  tx_thread_sleep(MS_TO_TICK(100));  /* reset        */
  mpu_write(0x6B, 0x01);  tx_thread_sleep(MS_TO_TICK(50));   /* wake up      */
  mpu_init_regs();                                           /* range/filter */
  tx_thread_sleep(MS_TO_TICK(50));

  /* ---- 3.2  Identity check ------------------------------------------- */
  HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, MPU_REG_WHOAMI, I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
  dbg_who = who;
  sprintf(banner, "WHO_AM_I=%d (expected 104)\r\n", who);
  uart_send_blocking(banner);

  /* ---- 3.3  Start the period generator ------------------------------- */
  tx_timer_activate(&sample_timer);

  /* ---- 3.4  Gyro bias calibration ------------------------------------
     Needed at EVERY power-up: the gyro offset changes with temperature
     and time, so it cannot be hard-coded. Keep the board STILL meanwhile. */
  {
    long sx = 0, sy = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
      tx_semaphore_get(&sample_sem, TX_WAIT_FOREVER);   /* wait 10 ms */
      mpu_read(&raw);
      sx += raw.gx;
      sy += raw.gy;
    }
    bx = (float)sx / (float)GYRO_CAL_SAMPLES;
    by = (float)sy / (float)GYRO_CAL_SAMPLES;
  }

  /* ---- 3.5  Initial angles (accelerometer only) ----------------------
     The calibration is applied here too; otherwise the filter would
     start from a wrong initial point. */
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

  /* ---- 3.6  MAIN LOOP : 100 Hz --------------------------------------- */
  while (1)
  {
    /* Wait for the start of the period. The thread uses no CPU here. */
    tx_semaphore_get(&sample_sem, TX_WAIT_FOREVER);

    mpu_read(&raw);

    /* --- Convert raw values to physical units ------------------------ */
    float fgx = (raw.gx - bx) / GYRO_LSB;            /* dps */
    float fgy = (raw.gy - by) / GYRO_LSB;

    /* Accelerometer: subtract the offset first, then divide by its own scale. */
    float fax = (raw.ax - ACC_OFF_X) / ACC_SCL_X;    /* g */
    float fay = (raw.ay - ACC_OFF_Y) / ACC_SCL_Y;
    float faz = (raw.az - ACC_OFF_Z) / ACC_SCL_Z;

    /* Angle from the accelerometer */
    float r_acc = atan2f(fay, faz) * RAD2DEG;
    float p_acc = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * RAD2DEG;

    /* --- MEASUREMENT TRUST (adaptive R) ------------------------------
       Tell the Kalman filter how much to trust this measurement.
       Large R -> ignore the measurement, trust the gyro.              */

    /* |a| : should be 1.000 when only gravity acts. Deviation = motion. */
    float a_mag = sqrtf(fax * fax + fay * fay + faz * faz);

    /* Roll observability: share of gravity in the Y-Z plane.
       pitch=0 -> 1.0 (perfect),  pitch=90 -> 0.0 (no information) */
    float g_yz  = sqrtf(fay * fay + faz * faz);

    float R_roll, R_pitch;

    if (fabsf(a_mag - 1.0f) > ACC_TRUST_BAND)
    {
      R_roll  = KF_R_BLIND;              /* in motion, trust neither axis */
      R_pitch = KF_R_BLIND;
    }
    else
    {
      R_pitch = KF_R_MEASURE;            /* pitch is observable at any angle */

      if (g_yz < ROLL_BLIND_GYZ)
        R_roll = KF_R_BLIND;             /* close to vertical: roll unreadable */
      else
        R_roll = KF_R_MEASURE / (g_yz * g_yz);
    }

    kalman_set_R(&kf_roll,  R_roll);
    kalman_set_R(&kf_pitch, R_pitch);

    /* --- KALMAN FILTER ----------------------------------------------- */
    roll  = kalman_update(&kf_roll,  r_acc, fgx, dt);
    pitch = kalman_update(&kf_pitch, p_acc, fgy, dt);

    /* --- Debug variables (Live Expressions) -------------------------- */
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

    /* --- Put the result into the queue ------------------------------- */
    msg.ax       = raw.ax;
    msg.ay       = raw.ay;
    msg.az       = raw.az;
    msg.gx       = raw.gx;
    msg.gy       = raw.gy;
    msg.gz       = raw.gz;
    msg.roll100  = (int16_t)(roll  * 100.0f);
    msg.pitch100 = (int16_t)(pitch * 100.0f);

    /* TX_NO_WAIT: if the queue is full (UART cannot keep up), drop this sample.
       NEVER stall the sampling - the filter timing must not be disturbed. */
    tx_queue_send(&imu_queue, &msg, TX_NO_WAIT);
  }
}

/* ============================================================================
 *  PART 4 : UART THREAD  -  send the CSV line
 * ==========================================================================*/
static void uart_thread_entry(ULONG arg)
{
  imu_msg_t msg;
  char      line[80];
  int       len;

  (void)arg;

  while (1)
  {
    /* Sleeps here while the queue is empty, uses no CPU. */
    tx_queue_receive(&imu_queue, &msg, TX_WAIT_FOREVER);

    len = sprintf(line, "%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                  msg.ax, msg.ay, msg.az,
                  msg.gx, msg.gy, msg.gz,
                  msg.roll100, msg.pitch100);

    /* Send with interrupts: the CPU does not busy-wait.
       'line' lives on the thread's stack; since we wait on the semaphore
       below, it stays valid until the transfer is finished. */
    if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)line, len) == HAL_OK)
    {
      tx_semaphore_get(&uart_tx_done, MS_TO_TICK(200));   /* recover if stuck */
    }
  }
}

/* Called from the USART2 interrupt when a transfer ends -> wakes UART_Thread */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    tx_semaphore_put(&uart_tx_done);
  }
}

/* ============================================================================
 *  PART 5 : SETUP  -  all ThreadX objects are created here
 * ==========================================================================*/
/**
  * @brief  Creates the ThreadX application objects.
  * @param  memory_ptr: byte pool created by CubeMX
  * @retval TX_SUCCESS or an error code
  * @note   Runs before the scheduler starts.
  *         NO HAL calls in here (interrupts are disabled).
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL *)memory_ptr;
  CHAR *ptr;

  /* --- 5.1  Queue : IMU -> UART -------------------------------------- */
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

  /* --- 5.2  Semaphores ------------------------------------------------ */
  /* Initial value 0: both sides wait for an "event" first. */
  if (tx_semaphore_create(&sample_sem, "Sample Tick", 0) != TX_SUCCESS)
  {
    return TX_SEMAPHORE_ERROR;
  }
  if (tx_semaphore_create(&uart_tx_done, "UART Done", 0) != TX_SUCCESS)
  {
    return TX_SEMAPHORE_ERROR;
  }

  /* --- 5.3  Period timer (10 ms) -------------------------------------- */
  /* TX_NO_ACTIVATE: IMU_Thread starts it itself after setting up the sensor. */
  if (tx_timer_create(&sample_timer, "Sample Timer", sample_timer_cb, 0,
                      SAMPLE_TICKS, SAMPLE_TICKS, TX_NO_ACTIVATE) != TX_SUCCESS)
  {
    return TX_TIMER_ERROR;
  }

  /* --- 5.4  IMU Thread (high priority) -------------------------------- */
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

  /* --- 5.5  UART Thread (low priority) -------------------------------- */
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
 *  PART 6 : KERNEL ENTRY
 * ==========================================================================*/
void MX_ThreadX_Init(void)
{
  tx_kernel_enter();   /* never returns */
}
