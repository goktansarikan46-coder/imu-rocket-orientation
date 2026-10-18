# IMU Rocket Orientation

Real-time **roll** and **pitch** estimation for a model rocket, running on an
STM32C092RC (Cortex-M0+) with an MPU-6050 IMU.

- **ThreadX RTOS**, two threads, fixed 100 Hz sampling
- **2-state Kalman filter** (angle + gyro bias) per axis
- **Adaptive measurement noise (R)**: the filter stops trusting the accelerometer
  during acceleration (e.g. motor burn) and near ±90° pitch
- Angles and raw sensor data are streamed over UART as CSV and shown as a live
  3D rocket in **Python** or **MATLAB**

## Repository layout

```
imu-rocket-orientation/
├── Firmware/RTOS_MPU6050/        STM32CubeIDE project (CubeMX .ioc, HAL, ThreadX)
│   ├── Core/Src/app_threadx.c    MPU-6050 driver, threads, adaptive R, CSV output
│   ├── Core/Src/kalman.c         2-state Kalman filter
│   ├── Core/Inc/kalman.h         Filter API and tuning constants
│   ├── Drivers/                  STM32C0 HAL and CMSIS (generated)
│   └── Middlewares/              Azure RTOS ThreadX (generated)
├── Python/
│   ├── MPU6050_Visualization.py  Live 3D rocket + roll/pitch plots, optional CSV logging
│   └── deneme.csv                Sample recording (~970 rows at 100 Hz)
├── Matlab/
│   └── mpu-6050-visualization.m  Same live view in MATLAB
├── Docs/                         Course reports and slides (in Turkish)
└── README.md
```

## Hardware

| Part | Notes |
|---|---|
| NUCLEO-C092RC | STM32C092RC, Cortex-M0+ |
| MPU-6050 | 3-axis accelerometer + 3-axis gyroscope, I²C address 0x68 |
| USB cable | ST-Link virtual COM port carries the UART data to the PC |

| Signal | MCU pin | Peripheral |
|---|---|---|
| MPU-6050 SDA | PA6 | I2C2 |
| MPU-6050 SCL | PA7 | I2C2 |
| UART TX / RX | PA2 / PA3 | USART2, 115200 baud |

Power the MPU-6050 from 3.3 V and share GND. Add 4.7 kΩ pull-ups on SDA/SCL if
your breakout board has none.

## Getting started

### Firmware

1. Open STM32CubeIDE and use **File → Import → Existing Projects into Workspace**,
   then select `Firmware/RTOS_MPU6050`.
2. Build and flash.
3. **Keep the board still for the first 10 seconds** after reset. The gyro bias
   is measured during this time.

### Python viewer

```bash
pip install pyserial pyqtgraph PyQt5 numpy
python Python/MPU6050_Visualization.py
```

Set `PORT` at the top of the script (`"COM3"`, `/dev/ttyACM0`, or `None` to pick
the first port found). `LOG_CSV` sets the file name for saving the incoming data.

### MATLAB viewer

Set `PORT` in `Matlab/mpu-6050-visualization.m` (find it with
`serialportlist("available")`) and run the script.

## Serial data format

After start-up the board prints a `WHO_AM_I=104` check line, then a header,
then 100 lines per second:

```
ax,ay,az,gx,gy,gz,roll100,pitch100
119,624,17690,176,121,-82,65,192
```

| Field | Meaning |
|---|---|
| `ax, ay, az` | Raw accelerometer values (LSB, ±2 g range) |
| `gx, gy, gz` | Raw gyroscope values (LSB, ±500 °/s range) |
| `roll100`, `pitch100` | Filtered angle × 100 in degrees, so `-1543` means `-15.43°` |

Angles are sent as integers to avoid `printf` float support on the small MCU.
Raw values are included so that other filters can be tried offline on the same
recording.

## How it works

### Firmware architecture

```
ThreadX timer (10 ms) ──► semaphore ──► IMU thread (priority 5) ──► queue (8) ──► UART thread (priority 10) ──► USART2
                                         read 14 bytes over I²C                    sprintf + interrupt-driven TX
                                         calibrate, Kalman filter
```

- **Timer → semaphore.** A ThreadX timer fires every tick (10 ms) and puts a
  semaphore with a ceiling of 1. If the IMU thread is ever late, ticks do not
  pile up and the thread does not run several times back to back.
- **Fixed `dt`.** `dt` comes from the timer period (0.01 s), not from measuring
  time between loops. A measured `dt` would include jitter and add integration
  error to the gyro angle.
- **Sampling never waits for UART.** The IMU thread sends to the queue with
  `TX_NO_WAIT`. If the UART falls behind, a sample is dropped instead of
  delaying the filter.
- **UART does not block the CPU.** The UART thread uses `HAL_UART_Transmit_IT`
  and sleeps on a semaphore that the TX-complete interrupt releases.

### Sensor setup and calibration

- MPU-6050 is configured for 100 Hz output, 42 Hz digital low-pass filter,
  ±500 °/s gyro and ±2 g accelerometer.
- **Gyro bias:** averaged over 1000 samples (10 s) at every power-up, because
  it changes with temperature and cannot be hard-coded.
- **Accelerometer:** offset and scale for each axis come from a 6-position test
  (`offset = (pos + neg) / 2`, `scale = (pos − neg) / 2`). These constants are
  specific to this sensor.

### Kalman filter

Each axis runs a 2-state filter: state = [angle, gyro bias].

- **Predict:** integrate the bias-corrected gyro rate and grow the covariance `P`.
- **Update:** correct with the accelerometer angle. The Kalman gain decides how
  much to trust it, and the gyro bias is learned online.
- For roll, the innovation is wrapped to ±180°, so +179° vs −179° is a 2° error,
  not 358°.

Tuning (`kalman.h`): `Q_angle = 0.001`, `Q_bias = 0.003`, `R = 0.03`.

### Adaptive measurement noise

The accelerometer angle is only correct when gravity is the only acceleration.
Before each update the firmware sets `R`:

| Condition | Roll R | Pitch R |
|---|---|---|
| `\|a\|` differs from 1 g by more than 0.15 g (motion, thrust) | 10 (ignore) | 10 (ignore) |
| Gravity in the Y-Z plane `g_yz < 0.15` (near ±90° pitch) | 10 (ignore) | 0.03 |
| Otherwise | `0.03 / g_yz²` | 0.03 |

When the measurement is ignored, the filter runs on the gyro alone. Roll trust
also drops smoothly as pitch grows, because roll becomes less observable from
gravity.

## Known limitations

- **No yaw.** An IMU without a magnetometer cannot hold a yaw reference.
- **Gimbal lock at ±90° pitch.** Euler angles are used; a quaternion filter would fix this.
- **Timer-driven, blocking I²C reads.** The MPU-6050 INT pin and DMA are not used yet.
- **Text protocol.** CSV is easy to read but large; there is no timestamp or checksum.
- **Raw-angle overlay in Python** uses the nominal 16384 LSB/g scale, without the
  firmware's calibration constants.

## Roadmap

- [ ] Measure sensor noise with Allan variance and derive Q and R from it
- [ ] Sample on the MPU-6050 data-ready interrupt, I²C/UART with DMA, binary protocol with CRC
- [ ] Quaternion Mahony filter (removes gimbal lock)
- [ ] Unit tests for the filters (GoogleTest) and GitHub Actions CI
- [ ] Magnetometer for yaw

## License

MIT — see [LICENSE](LICENSE).
