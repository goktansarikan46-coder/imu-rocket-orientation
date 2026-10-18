"""
Plot a recorded CSV: accelerometer-only angle vs. the firmware's Kalman output.

The accelerometer angle is recomputed here with the same formula and the same
6-position calibration constants as the firmware (app_threadx.c), so the two
curves are directly comparable.

Usage:    python Python/plot_recording.py [recording.csv] [output.png]
Defaults: Python/sample_recording.csv -> Docs/images/roll_pitch_sample.png
Requires: pip install numpy matplotlib
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SAMPLE_RATE_HZ = 100.0

# Same constants as Firmware/RTOS_MPU6050/Core/Src/app_threadx.c
ACC_OFF = np.array([651.5, 415.5, 800.5])        # LSB
ACC_SCL = np.array([16359.5, 16393.5, 16863.5])  # LSB/g


def accel_angles(raw_acc):
    """Roll and pitch [deg] from the calibrated accelerometer only."""
    ax, ay, az = ((raw_acc - ACC_OFF) / ACC_SCL).T
    roll = np.degrees(np.arctan2(ay, az))
    pitch = np.degrees(np.arctan2(-ax, np.sqrt(ay * ay + az * az)))
    return roll, pitch


def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "Python" / "sample_recording.csv"
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "Docs" / "images" / "roll_pitch_sample.png"

    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    t = np.arange(len(data)) / SAMPLE_RATE_HZ
    roll_acc, pitch_acc = accel_angles(data[:, 0:3])
    roll_kf, pitch_kf = data[:, 6] / 100.0, data[:, 7] / 100.0

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for ax, acc, kf, name in ((axes[0], roll_acc, roll_kf, "Roll"),
                              (axes[1], pitch_acc, pitch_kf, "Pitch")):
        ax.plot(t, acc, color="0.65", linewidth=0.9, label="Accelerometer only")
        ax.plot(t, kf, color="tab:blue", linewidth=1.6, label="Kalman filter (firmware)")
        ax.set_ylabel(f"{name} [deg]")
        ax.grid(True, alpha=0.3)
        rms = np.sqrt(np.mean((acc - kf) ** 2))
        ax.set_title(f"{name}  -  RMS difference {rms:.2f} deg", loc="left", fontsize=10)
    axes[0].legend(loc="lower left")
    axes[1].set_xlabel("Time [s]")
    fig.suptitle(f"{csv_path.name}: {len(data)} samples at {SAMPLE_RATE_HZ:.0f} Hz")
    fig.tight_layout()

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    print("Saved", out_path)


if __name__ == "__main__":
    main()
