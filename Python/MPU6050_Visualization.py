"""
==========================================================================
 3D ROCKET VISUALIZATION
 STM32 + MPU-6050  ->  serial port  ->  Python
==========================================================================

Line format sent by the board (CSV, ~100 lines per second):

    ax,ay,az,gx,gy,gz,roll100,pitch100

roll100 / pitch100 = angle * 100   (the firmware sends integers, so divide by 100)


DATA FLOW
---------
    [Serial port]
         |   background reader thread                  (~100 Hz)
         v
    latest   <- always holds the most recent line
         |   screen timer (QTimer)                     (~60 FPS)
         v
    [Smoothing] -> [3D rocket] + [roll/pitch plots]

Why two different rates?
    The sensor produces 100 Hz and the screen draws at 60 FPS. Reading in a
    separate thread means lines never pile up in the serial buffer even if
    drawing slows down; the screen always uses the newest value.


Install: pip install pyserial pyqtgraph PyQt5 numpy
Run:     python Python/MPU6050_Visualization.py
"""

import sys
import math
import time
import threading

import numpy as np
import serial
import serial.tools.list_ports

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets


# ==========================================================================
# 1) SETTINGS   -  start here
# ==========================================================================

PORT      = None      # None -> pick the first port found. Or set it: "COM3" / "/dev/ttyACM0"
BAUD      = 115200
TAU       = 0.05      # smoothing time constant [s]. Increase if jittery, decrease if laggy.
RSIGN     = +1        # set to -1 if roll turns the wrong way
PSIGN     = +1        # set to -1 if pitch tilts the wrong way
N_SAMPLES = 400       # samples visible in the plots (~4 seconds)
LOG_CSV   = None      # set a file name to record the stream, e.g. LOG_CSV = "recording.csv"

# Accelerometer calibration, same constants as the firmware (app_threadx.c)
ACC_OFF = (651.5, 415.5, 800.5)          # LSB
ACC_SCL = (16359.5, 16393.5, 16863.5)    # LSB/g

# Rocket dimensions (body axis +Z, centre ~ origin)
BODY_R     = 0.25
BASE_Z     = -2.40
SHOULDER_Z = +1.30    # where the nose cone starts
NOSE_Z     = +2.60

# Colours: 0..1 (r, g, b)
WHITE       = (0.92, 0.93, 0.96)
RED         = (0.90, 0.25, 0.25)
GREY        = (0.55, 0.55, 0.60)
BLUE        = (0.20, 0.52, 0.95)
FLAME_OUTER = (1.00, 0.42, 0.08)
FLAME_INNER = (1.00, 0.88, 0.45)


# ==========================================================================
# 2) SERIAL READER  (background thread)
# ==========================================================================

class SerialReader(threading.Thread):
    """Reads lines from the serial port continuously and keeps the latest valid one."""

    def __init__(self, port, baud, log_path=None):
        super().__init__(daemon=True)
        self.ser        = serial.Serial(port, baud, timeout=0.2)
        self.lock       = threading.Lock()   # two threads share these fields
        self.latest     = None               # [ax, ay, az, gx, gy, gz, roll100, pitch100]
        self.line_count = 0
        self.running    = True
        self.log = None
        if log_path:
            # buffering=1 -> every line is flushed to disk right away, so the
            # data is kept even if the program closes unexpectedly.
            self.log = open(log_path, "w", buffering=1)
            self.log.write("ax,ay,az,gx,gy,gz,roll100,pitch100\n")

    def run(self):
        """Runs when the thread starts. Independent of screen drawing."""
        self.ser.reset_input_buffer()
        while self.running:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            data = self._parse_line(raw)
            if data is None:
                continue
            with self.lock:
                self.latest = data
                self.line_count += 1

            # Write to the file OUTSIDE the lock; disk I/O is slow and holding
            # the lock while waiting would stall the other thread for nothing.
            if self.log:
                self.log.write(",".join(str(x) for x in data) + "\n")

    @staticmethod
    def _parse_line(raw_bytes):
        """Converts one line to 8 integers. Returns None for broken or header lines."""
        text = raw_bytes.decode("ascii", "ignore").strip()
        parts = text.split(",")
        if len(parts) != 8:
            return None                 # drops lines like "WHO_AM_I=104"
        try:
            return [int(p) for p in parts]
        except ValueError:
            return None                 # drops the "ax,ay,az,..." header line

    def read(self):
        """Called by the screen side: (latest, total_lines)."""
        with self.lock:
            return self.latest, self.line_count

    def stop(self):
        self.running = False
        time.sleep(0.25)
        try:
            self.ser.close()
        except Exception:
            pass
        if self.log:
            self.log.close()


# ==========================================================================
# 3) SMALL HELPERS
# ==========================================================================

class Smoother:
    """
    Exponential smoothing (low-pass):   value += k * (target - value)

    k is not fixed; it is computed from the frame time: k = 1 - exp(-dt/tau)
    so the smoothing speed stays the same even if the computer slows down.

    With wrap_angle=True the -180/+180 crossing takes the short way
    (179 -> -179 is 2 degrees, not 358).
    """

    def __init__(self, tau, wrap_angle=False):
        self.tau = tau
        self.wrap_angle = wrap_angle
        self.value = 0.0

    def update(self, target, dt):
        k = 1.0 - math.exp(-dt / self.tau)
        diff = target - self.value
        if self.wrap_angle:
            diff = (diff + 180.0) % 360.0 - 180.0
        self.value += k * diff
        return self.value


class RingBuffer:
    """Sliding window for the plots: new sample enters on the right, oldest leaves on the left."""

    def __init__(self, n):
        self.data = np.zeros(n)

    def push(self, x):
        self.data[:-1] = self.data[1:]   # in-place shift (no new array)
        self.data[-1] = x


def accel_angles(ax, ay, az):
    """
    Roll/pitch from the accelerometer only, using the direction of gravity.
    Same formula and calibration as the firmware; shown next to the filtered result.
    """
    fax = (ax - ACC_OFF[0]) / ACC_SCL[0]
    fay = (ay - ACC_OFF[1]) / ACC_SCL[1]
    faz = (az - ACC_OFF[2]) / ACC_SCL[2]
    roll  = math.degrees(math.atan2(fay, faz))
    pitch = math.degrees(math.atan2(-fax, math.sqrt(fay * fay + faz * faz)))
    return roll, pitch


# ==========================================================================
# 4) 3D ROCKET MODEL
# ==========================================================================

class Rocket:
    """
    Holds all rocket parts and rotates them with the same rotation matrix.

    Each part is stored as an (item, base_transform) pair:
        base_transform = fixed place of the part inside the rocket body
        rot            = current attitude of the rocket
        final matrix   = rot * base_transform

    The flame is kept separately because its length changes every frame (flicker).
    """

    def __init__(self, view):
        self.view   = view
        self.parts  = []      # (item, fixed base transform)
        self.flames = []      # (item, base length)

        self._build_body()
        self._build_fins()
        self._build_flame()

    # ---- setup ----------------------------------------------------------
    def _add(self, item, base=None):
        self.parts.append((item, base if base is not None else pg.Transform3D()))
        self.view.addItem(item)

    def _build_body(self):
        self._add(*self._cylinder(BODY_R, BODY_R, SHOULDER_Z - BASE_Z, WHITE, BASE_Z))
        self._add(*self._cylinder(BODY_R, 0.0,    NOSE_Z - SHOULDER_Z, RED,   SHOULDER_Z))
        self._add(*self._cylinder(0.0,    BODY_R, 0.20,                GREY,  BASE_Z))

    def _build_fins(self):
        for angle in (0, 120, 240):
            md = self._fin_mesh(angle, r_root=BODY_R, r_tip=0.80,
                                z=BASE_Z, h_root=1.00, h_tip=0.45)
            self._add(gl.GLMeshItem(meshdata=md, smooth=False, shader="shaded",
                                    color=BLUE + (1.0,)))

    def _build_flame(self):
        """
        Two nested cones: a wide orange outer flame and a thin yellow core.
        'additive' blending brightens where they overlap and gives a glowing look.
        """
        layers = [
            # radius,          length, color,       alpha
            (BODY_R * 0.95,    1.70,   FLAME_OUTER, 0.55),
            (BODY_R * 0.50,    0.95,   FLAME_INNER, 0.90),
        ]
        for radius, length, color, alpha in layers:
            md = gl.MeshData.cylinder(rows=1, cols=20, radius=[radius, 0.0], length=1.0)
            item = gl.GLMeshItem(meshdata=md, smooth=True, shader="balloon",
                                 color=color + (alpha,), glOptions="additive")
            self.view.addItem(item)
            self.flames.append((item, length))

    # ---- geometry helpers -----------------------------------------------
    @staticmethod
    def _cylinder(r_bottom, r_top, length, color, z_base):
        """Creates a cylinder or cone (r_top=0 -> cone). Body axis +Z."""
        md = gl.MeshData.cylinder(rows=1, cols=24, radius=[r_bottom, r_top], length=length)
        item = gl.GLMeshItem(meshdata=md, smooth=True, shader="shaded", color=color + (1.0,))
        base = pg.Transform3D()
        base.translate(0, 0, z_base)
        return item, base

    @staticmethod
    def _fin_mesh(angle_deg, r_root, r_tip, z, h_root, h_tip):
        """One fin: a trapezoid pointing out of the body, 2 triangles."""
        a = math.radians(angle_deg)
        ux, uy = math.cos(a), math.sin(a)
        profile = [(r_root, z), (r_root, z + h_root), (r_tip, z + h_tip), (r_tip, z)]
        verts = np.array([[r * ux, r * uy, zz] for (r, zz) in profile], dtype=float)
        faces = np.array([[0, 1, 2], [0, 2, 3]])
        return gl.MeshData(vertexes=verts, faces=faces)

    @staticmethod
    def _flame_base(length):
        """
        The cone points up (0..1). It is first scaled to its length, then turned
        180 degrees to point down, and finally moved to the tail.
        (Matrix products read right to left: T * R * S)
        """
        base = pg.Transform3D()
        base.translate(0, 0, BASE_Z)
        base.rotate(180, 1, 0, 0)
        base.scale(1, 1, length)
        return base

    @staticmethod
    def _flicker(t_s):
        """Sum of two sines at different speeds -> natural flicker that does not look periodic."""
        return 1.0 + 0.14 * math.sin(t_s * 23.0) + 0.07 * math.sin(t_s * 41.0)

    # ---- called every frame ---------------------------------------------
    def set_attitude(self, roll_deg, pitch_deg, t_s):
        """
        Rotation matrix = Ry(pitch) * Rx(roll)
        (Same order as the MATLAB code: roll is applied first, then pitch.)
        """
        rot = pg.Transform3D()
        rot.rotate(pitch_deg, 0, 1, 0)
        rot.rotate(roll_deg,  1, 0, 0)

        for item, base in self.parts:
            item.setTransform(rot * base)

        k = self._flicker(t_s)
        for item, length in self.flames:
            item.setTransform(rot * self._flame_base(length * k))


# ==========================================================================
# 5) ANGLE PLOT  (filtered + raw accelerometer)
# ==========================================================================

class AnglePlot:
    """Scrolling plot for one angle. Coloured line = filtered, grey = raw accelerometer."""

    def __init__(self, title, color_255, y_limit, n):
        self.widget = pg.PlotWidget(title=title)
        self.widget.setBackground("k")
        self.widget.setYRange(-y_limit, y_limit)
        self.widget.setXRange(0, n)
        self.widget.showGrid(x=True, y=True, alpha=0.25)
        self.widget.setMouseEnabled(x=False, y=False)
        self.widget.getPlotItem().hideAxis("bottom")
        self.widget.getPlotItem().titleLabel.setText(
            title, color=pg.mkColor(color_255), size="11pt")

        self.raw_curve  = self.widget.plot(pen=pg.mkPen((130, 135, 145, 140), width=1))
        self.filt_curve = self.widget.plot(pen=pg.mkPen(color_255, width=2))
        self.raw_buf    = RingBuffer(n)
        self.filt_buf   = RingBuffer(n)

    def update(self, filtered, raw):
        self.filt_buf.push(filtered)
        self.raw_buf.push(raw)
        self.filt_curve.setData(self.filt_buf.data)
        self.raw_curve.setData(self.raw_buf.data)


# ==========================================================================
# 6) MAIN WINDOW
# ==========================================================================

class Window(QtWidgets.QWidget):

    def __init__(self, reader):
        super().__init__()
        self.reader = reader

        self.setWindowTitle("Rocket - Attitude")
        self.resize(1280, 760)
        self.setStyleSheet("background-color:#000;")

        self._build_scene()
        self._build_panel()
        self._build_layout()

        # Smoothers: roll wraps around (-180/+180), pitch does not.
        self.roll_smoother  = Smoother(TAU, wrap_angle=True)
        self.pitch_smoother = Smoother(TAU, wrap_angle=False)

        self.roll_target = self.pitch_target = 0.0
        self.roll_raw    = self.pitch_raw    = 0.0

        self.t_prev  = time.perf_counter()
        self.t_start = time.perf_counter()
        self.frames  = 0

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.on_frame)
        self.timer.start(16)          # 16 ms -> ~60 FPS

    # ---- UI setup -------------------------------------------------------
    def _build_scene(self):
        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor("k")
        self.view.opts["distance"]  = 13
        self.view.opts["elevation"] = 16
        self.view.opts["azimuth"]   = -40

        grid = gl.GLGridItem()
        grid.setSize(12, 12)
        grid.setSpacing(1, 1)
        grid.translate(0, 0, -4.5)
        self.view.addItem(grid)

        self.rocket = Rocket(self.view)

    def _build_panel(self):
        self.angle_label = QtWidgets.QLabel("ROLL   ---\nPITCH  ---")
        self.angle_label.setStyleSheet(
            "color:#f2f4f8; font-family:Consolas,monospace; font-size:26px;")

        pg.setConfigOptions(antialias=True)
        self.roll_plot  = AnglePlot("ROLL",  (60, 205, 255), 180, N_SAMPLES)
        self.pitch_plot = AnglePlot("PITCH", (255, 155, 50),  90, N_SAMPLES)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setStyleSheet(
            "color:#737d8c; font-family:Consolas,monospace; font-size:12px;")

    def _build_layout(self):
        left = QtWidgets.QVBoxLayout()
        left.addWidget(self.angle_label)
        left.addWidget(self.view, 1)

        right = QtWidgets.QVBoxLayout()
        right.addWidget(self.status_label)
        right.addWidget(self.roll_plot.widget, 1)
        right.addWidget(self.pitch_plot.widget, 1)

        main_layout = QtWidgets.QHBoxLayout(self)
        main_layout.addLayout(left, 62)
        main_layout.addLayout(right, 38)

    # ---- per-frame loop -------------------------------------------------
    def on_frame(self):
        now = time.perf_counter()
        dt = now - self.t_prev
        self.t_prev = now
        elapsed = now - self.t_start

        # 1) Take the latest line
        data, lines = self.reader.read()
        if data is not None:
            self.roll_target  = RSIGN * data[6] / 100.0
            self.pitch_target = PSIGN * data[7] / 100.0
            r_raw, p_raw = accel_angles(data[0], data[1], data[2])
            self.roll_raw, self.pitch_raw = RSIGN * r_raw, PSIGN * p_raw
        else:
            lines = 0

        # 2) Smooth (visual only; the real filtering runs on the board)
        roll  = self.roll_smoother.update(self.roll_target, dt)
        pitch = self.pitch_smoother.update(self.pitch_target, dt)

        # 3) Draw
        self.rocket.set_attitude(roll, pitch, elapsed)
        self.roll_plot.update(roll, self.roll_raw)
        self.pitch_plot.update(pitch, self.pitch_raw)

        # 4) Numeric readouts
        self.frames += 1
        fps = self.frames / elapsed
        self.angle_label.setText("ROLL  %+7.2f\nPITCH %+7.2f" % (roll, pitch))
        self.status_label.setText(
            "%.0f fps | %d lines | grey line: raw accelerometer angle" % (fps, lines))

    def closeEvent(self, event):
        self.timer.stop()
        self.reader.stop()
        event.accept()


# ==========================================================================
# 7) START
# ==========================================================================

def pick_port():
    if PORT:
        return PORT
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial port found. Plug in the board or set PORT by hand.")
        sys.exit(1)
    print("Ports found:")
    for p in ports:
        print("   ", p.device, "-", p.description)
    print("Using:", ports[0].device)
    return ports[0].device


def main():
    reader = SerialReader(pick_port(), BAUD, LOG_CSV)
    reader.start()

    pg.mkQApp("Rocket")
    window = Window(reader)
    window.show()
    pg.exec()


if __name__ == "__main__":
    main()
