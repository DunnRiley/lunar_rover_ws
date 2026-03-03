#!/usr/bin/env python3
"""
Lunar Rover Mission Control GUI  —  rover_control_gui.py
UPDATED: Tank drive — dual speed sliders (left/right independent) + servo position control.
"""

import os, sys, math, subprocess, threading, time

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit, QSlider,
    QSizePolicy, QFrame, QLineEdit, QDial
)
from PyQt5.QtGui  import QFont, QColor, QPainter, QBrush, QPen, QLinearGradient
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QRect, QPoint

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int8 as RosInt8, Bool, String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ── CONFIG ────────────────────────────────────────────────────────────────
MINIPC_USER = "cheese"
MINIPC_IP   = "192.168.0.102"
MINIPC_WS   = "~/lunar_rover_ws"
RVIZ_CONFIG = os.path.expanduser("~/lunar_rover_ws/laptop_stream.rviz")

# ── D-pad servo angles (must match joy_to_arduino.py) ────────────────────
SERVO_UP    = 90
SERVO_LEFT  = 0
SERVO_RIGHT = 180


# ═════════════════════════════════════════════════════════════════════════
# SERVO DIAL WIDGET
# ═════════════════════════════════════════════════════════════════════════

class ServoGauge(QWidget):
    """
    Arc gauge showing servo position 0–180°.
    The needle sweeps a 180° arc (left=0°, up=90°, right=180°).
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self._angle = 90          # current servo angle 0–180
        self._target = 90         # for smooth animation
        self._anim_timer = QTimer()
        self._anim_timer.timeout.connect(self._step_anim)
        self.setMinimumSize(140, 90)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_angle(self, angle: int):
        self._target = max(0, min(180, angle))
        if not self._anim_timer.isActive():
            self._anim_timer.start(16)   # ~60 fps

    def _step_anim(self):
        diff = self._target - self._angle
        if abs(diff) < 0.5:
            self._angle = self._target
            self._anim_timer.stop()
        else:
            self._angle += diff * 0.18
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx    = w // 2
        cy    = h - 10
        r_out = min(cx - 4, h - 14)
        r_in  = int(r_out * 0.60)

        # ── Background arc ────────────────────────────────────────────────
        arc_pen = QPen(QColor(30, 38, 52), 10)
        arc_pen.setCapStyle(Qt.RoundCap)
        p.setPen(arc_pen)
        p.drawArc(QRect(cx - r_out, cy - r_out, r_out * 2, r_out * 2),
                  0 * 16, 180 * 16)

        # ── Filled arc (progress) ─────────────────────────────────────────
        grad = QLinearGradient(cx - r_out, cy, cx + r_out, cy)
        grad.setColorAt(0.0, QColor(42, 128, 192))
        grad.setColorAt(0.5, QColor(80, 200, 160))
        grad.setColorAt(1.0, QColor(232, 160, 48))
        fill_pen = QPen(grad, 10)
        fill_pen.setCapStyle(Qt.RoundCap)
        p.setPen(fill_pen)
        # arc spans 0° (right=180°) to current angle
        # Qt arc: 0° = 3 o'clock, positive = CCW.  Our 0° servo = left = 180° Qt.
        # servo 0   → Qt start=180°, span=0
        # servo 90  → Qt start=180°, span=90°  (to 90° Qt = top)
        # servo 180 → Qt start=180°, span=180°
        span = int(self._angle)
        if span > 0:
            p.drawArc(QRect(cx - r_out, cy - r_out, r_out * 2, r_out * 2),
                      180 * 16, span * 16)

        # ── Tick marks at 0, 45, 90, 135, 180 ────────────────────────────
        tick_pen = QPen(QColor(50, 65, 85), 2)
        p.setPen(tick_pen)
        for deg in (0, 45, 90, 135, 180):
            rad = math.radians(180 - deg)   # convert servo→Qt angle
            x1 = cx + (r_out + 2) * math.cos(rad)
            y1 = cy - (r_out + 2) * math.sin(rad)
            x2 = cx + (r_out - 8) * math.cos(rad)
            y2 = cy - (r_out - 8) * math.sin(rad)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

        # ── Needle ────────────────────────────────────────────────────────
        needle_rad = math.radians(180 - self._angle)
        nx = cx + r_in * math.cos(needle_rad)
        ny = cy - r_in * math.sin(needle_rad)
        needle_pen = QPen(QColor(232, 160, 48), 3)
        needle_pen.setCapStyle(Qt.RoundCap)
        p.setPen(needle_pen)
        p.drawLine(cx, cy, int(nx), int(ny))

        # ── Hub dot ───────────────────────────────────────────────────────
        p.setBrush(QBrush(QColor(232, 160, 48)))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPoint(cx, cy), 5, 5)

        # ── Angle label ───────────────────────────────────────────────────
        p.setPen(QPen(QColor(192, 204, 224)))
        font = QFont("Courier New", 9, QFont.Bold)
        p.setFont(font)
        text = f"{int(round(self._angle))}°"
        p.drawText(QRect(0, 0, w, h - 2), Qt.AlignHCenter | Qt.AlignBottom, text)


# ═════════════════════════════════════════════════════════════════════════
# TELEOP PUBLISHER
# ═════════════════════════════════════════════════════════════════════════

class TeleopPublisher(QThread):
    status_changed      = pyqtSignal(str)
    speed_left_changed  = pyqtSignal(float)
    speed_right_changed = pyqtSignal(float)
    servo_changed       = pyqtSignal(int)

    # ── Xbox USB axis/button mapping ──────────────────────────────────────
    AXIS_LEFT    = 1    # Left  stick Y  → LEFT  wheels
    AXIS_RIGHT   = 4    # Right stick Y  → RIGHT wheels

    BTN_RIGHT_UP   = 3  # Y — right speed UP
    BTN_RIGHT_DOWN = 0  # A — right speed DOWN
    BTN_LEFT_UP    = 2  # X — left  speed UP
    BTN_LEFT_DOWN  = 1  # B — left  speed DOWN

    BTN_LB    = 4
    BTN_RB    = 5
    BTN_START = 7

    DPAD_AXIS_LR = 6
    DPAD_AXIS_UD = 7

    JOY_DEADZONE = 0.10
    SPEED_STEP   = 0.05

    def __init__(self):
        super().__init__()
        self._lock    = threading.Lock()
        self._running = False
        self._speed_left  = 0.5
        self._speed_right = 0.5
        self._node    = None
        self._pub     = None
        self._act_pub = None

        self._want_lin = 0.0
        self._want_ang = 0.0
        self._want_act = 0

        self._sent_lin = None
        self._sent_ang = None
        self._sent_act = None

        self._prev_btns   = {}
        self._prev_dpad_lr = 0.0
        self._prev_dpad_ud = 0.0

    # ── Public setters ────────────────────────────────────────────────────

    def set_speed_left(self, v: float):
        with self._lock:
            self._speed_left = max(0.05, min(1.0, v))

    def set_speed_right(self, v: float):
        with self._lock:
            self._speed_right = max(0.05, min(1.0, v))

    def emergency_stop(self):
        with self._lock:
            self._want_lin = 0.0
            self._want_ang = 0.0
            self._want_act = 0

    # ── Helpers ───────────────────────────────────────────────────────────

    def _dz(self, v):
        return v if abs(v) >= self.JOY_DEADZONE else 0.0

    def _rising(self, idx, cur):
        prev = self._prev_btns.get(idx, 0)
        self._prev_btns[idx] = cur
        return cur == 1 and prev == 0

    # ── /joy callback ─────────────────────────────────────────────────────

    def _joy_cb(self, msg):
        try:
            ax  = lambda i: msg.axes[i]    if i < len(msg.axes)    else 0.0
            btn = lambda i: msg.buttons[i] if i < len(msg.buttons) else 0

            # Per-side speed
            if self._rising(self.BTN_RIGHT_UP, btn(self.BTN_RIGHT_UP)):
                with self._lock:
                    self._speed_right = round(min(1.0, self._speed_right + self.SPEED_STEP), 2)
                    s = self._speed_right
                self.speed_right_changed.emit(s)
                self.status_changed.emit(f'RIGHT speed: {s:.2f}')

            if self._rising(self.BTN_RIGHT_DOWN, btn(self.BTN_RIGHT_DOWN)):
                with self._lock:
                    self._speed_right = round(max(0.05, self._speed_right - self.SPEED_STEP), 2)
                    s = self._speed_right
                self.speed_right_changed.emit(s)
                self.status_changed.emit(f'RIGHT speed: {s:.2f}')

            if self._rising(self.BTN_LEFT_UP, btn(self.BTN_LEFT_UP)):
                with self._lock:
                    self._speed_left = round(min(1.0, self._speed_left + self.SPEED_STEP), 2)
                    s = self._speed_left
                self.speed_left_changed.emit(s)
                self.status_changed.emit(f'LEFT  speed: {s:.2f}')

            if self._rising(self.BTN_LEFT_DOWN, btn(self.BTN_LEFT_DOWN)):
                with self._lock:
                    self._speed_left = round(max(0.05, self._speed_left - self.SPEED_STEP), 2)
                    s = self._speed_left
                self.speed_left_changed.emit(s)
                self.status_changed.emit(f'LEFT  speed: {s:.2f}')

            # Actuators
            lb  = btn(self.BTN_LB)
            rb  = btn(self.BTN_RB)
            act = 1 if lb else (-1 if rb else 0)

            # D-pad servo
            cur_lr = ax(self.DPAD_AXIS_LR)
            cur_ud = ax(self.DPAD_AXIS_UD)

            if cur_ud > 0.5 and self._prev_dpad_ud <= 0.5:
                self.servo_changed.emit(SERVO_UP)
                self.status_changed.emit(f'Servo → {SERVO_UP}° (centre)')
            if cur_lr < -0.5 and self._prev_dpad_lr >= -0.5:
                self.servo_changed.emit(SERVO_LEFT)
                self.status_changed.emit(f'Servo → {SERVO_LEFT}° (left)')
            if cur_lr > 0.5 and self._prev_dpad_lr <= 0.5:
                self.servo_changed.emit(SERVO_RIGHT)
                self.status_changed.emit(f'Servo → {SERVO_RIGHT}° (right)')

            self._prev_dpad_lr = cur_lr
            self._prev_dpad_ud = cur_ud

            # Tank drive: publish Twist so nav stack / logs still work
            raw_left  = self._dz(ax(self.AXIS_LEFT))
            raw_right = self._dz(ax(self.AXIS_RIGHT))
            with self._lock:
                sl = self._speed_left
                sr = self._speed_right
            left_f  = raw_left  * sl
            right_f = raw_right * sr
            lin = (left_f + right_f) / 2.0
            ang = (right_f - left_f) / 2.0   # approx twist

            with self._lock:
                self._want_lin = lin
                self._want_ang = ang
                self._want_act = act

        except Exception as e:
            self.status_changed.emit(f'Joy error: {e}')

    # ── Flush ─────────────────────────────────────────────────────────────

    def _flush(self):
        with self._lock:
            lin = self._want_lin
            ang = self._want_ang
            act = self._want_act

        if lin != self._sent_lin or ang != self._sent_ang:
            msg = Twist()
            msg.linear.x  = float(lin)
            msg.angular.z = float(ang)
            self._pub.publish(msg)
            self._sent_lin = lin
            self._sent_ang = ang

        if act != self._sent_act:
            m = RosInt8(); m.data = act
            self._act_pub.publish(m)
            self._sent_act = act

    # ── ROS thread ────────────────────────────────────────────────────────

    def run(self):
        if not ROS_AVAILABLE:
            self.status_changed.emit('ROS2 not available')
            return
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node('rover_laptop_teleop')

            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self._pub     = self._node.create_publisher(Twist,   '/cmd_vel',      qos)
            self._act_pub = self._node.create_publisher(RosInt8, '/actuator_cmd', qos)

            try:
                from sensor_msgs.msg import Joy
                self._node.create_subscription(Joy, '/joy', self._joy_cb, 10)
                self.status_changed.emit('Teleop active  ·  controller connected')
            except Exception:
                self.status_changed.emit('Teleop active  ·  no sensor_msgs')

            self._running = True
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._node)

            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.02)
                self._flush()

        except Exception as e:
            self.status_changed.emit(f'Teleop error: {e}')
        finally:
            self._running = False
            if self._node:
                try: self._node.destroy_node()
                except: pass

    def stop(self):
        self.emergency_stop()
        self._running = False
        self.quit()
        self.wait(2000)


# ═════════════════════════════════════════════════════════════════════════
# STATUS LED
# ═════════════════════════════════════════════════════════════════════════
class StatusLED(QLabel):
    COLORS = {
        "off":    QColor(40,  42,  50),
        "green":  QColor(60,  220, 80),
        "yellow": QColor(255, 200, 40),
        "red":    QColor(220, 60,  60),
    }
    def __init__(self, color="off", parent=None):
        super().__init__(parent)
        self.setFixedSize(14, 14)
        self.set_color(color)

    def set_color(self, color):
        self._color = color
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = self.COLORS.get(self._color, self.COLORS["off"])
        p.setBrush(QBrush(c))
        p.setPen(QPen(c.darker(150), 1))
        p.drawEllipse(1, 1, 12, 12)


# ═════════════════════════════════════════════════════════════════════════
# PROCESS CARD
# ═════════════════════════════════════════════════════════════════════════
class ProcessCard(QGroupBox):
    def __init__(self, title, cmd_fn, parent=None):
        super().__init__(title, parent)
        self._cmd_fn = cmd_fn
        self._proc   = None
        self._build()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setSpacing(4)
        row = QHBoxLayout()
        self.led = StatusLED("off")
        row.addWidget(self.led)
        self.btn = QPushButton("START")
        self.btn.setCheckable(True)
        self.btn.setFixedHeight(28)
        self.btn.clicked.connect(self._toggle)
        row.addWidget(self.btn)
        lay.addLayout(row)
        self.log = QLabel("—")
        self.log.setStyleSheet("color:#506070; font-size:8px;")
        self.log.setWordWrap(True)
        lay.addWidget(self.log)

    def _toggle(self, checked):
        if checked: self._start()
        else:       self._stop()

    def _start(self):
        cmd = self._cmd_fn()
        if not cmd:
            self.btn.setChecked(False)
            return
        self._proc = subprocess.Popen(cmd, shell=True)
        self.led.set_color("green")
        self.btn.setText("STOP")
        self.log.setText(f"PID {self._proc.pid}")

    def _stop(self):
        if self._proc:
            self._proc.terminate()
            self._proc = None
        self.led.set_color("off")
        self.btn.setText("START")
        self.btn.setChecked(False)
        self.log.setText("stopped")


# ═════════════════════════════════════════════════════════════════════════
# DUAL SPEED SLIDER WIDGET
# ═════════════════════════════════════════════════════════════════════════

class DualSpeedWidget(QGroupBox):
    """
    Side-by-side left and right speed sliders with individual labels.
    Emits left_changed(float) and right_changed(float).
    """
    left_changed  = pyqtSignal(float)
    right_changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__("SPEED  ·  tank drive independent", parent)
        self._build()

    def _build(self):
        root = QHBoxLayout(self)
        root.setSpacing(12)

        def make_side(label_text, color):
            col = QVBoxLayout()
            col.setSpacing(3)

            hdr = QLabel(label_text)
            hdr.setAlignment(Qt.AlignCenter)
            hdr.setStyleSheet(f"color:{color}; font-size:9px; font-weight:bold; letter-spacing:1px;")
            col.addWidget(hdr)

            slider = QSlider(Qt.Vertical)
            slider.setRange(5, 100)
            slider.setValue(50)
            slider.setFixedHeight(90)
            slider.setStyleSheet(f"""
                QSlider::groove:vertical {{
                    background:#1a1e28; width:6px; border-radius:3px;
                }}
                QSlider::handle:vertical {{
                    background:{color}; width:14px; height:14px;
                    margin:-4px -4px; border-radius:7px;
                }}
                QSlider::sub-page:vertical {{
                    background:#1a1e28;
                }}
                QSlider::add-page:vertical {{
                    background:{color}44; border-radius:3px;
                }}
            """)

            val_lbl = QLabel("0.50")
            val_lbl.setAlignment(Qt.AlignCenter)
            val_lbl.setStyleSheet(f"color:{color}; font-size:11px; font-weight:bold;")

            slider_row = QHBoxLayout()
            slider_row.addStretch()
            slider_row.addWidget(slider)
            slider_row.addStretch()
            col.addLayout(slider_row)
            col.addWidget(val_lbl)

            return col, slider, val_lbl

        left_col,  self.left_slider,  self.left_val  = make_side("◀  LEFT",  "#50c878")
        right_col, self.right_slider, self.right_val = make_side("RIGHT  ▶", "#e8a030")

        # Divider
        div = QFrame()
        div.setFrameShape(QFrame.VLine)
        div.setStyleSheet("color:#1a2030;")

        root.addLayout(left_col)
        root.addWidget(div)
        root.addLayout(right_col)

        self.left_slider.valueChanged.connect(self._on_left)
        self.right_slider.valueChanged.connect(self._on_right)

        # Sync hint row
        sync_row = QHBoxLayout()
        self._sync_btn = QPushButton("⟺  Sync both")
        self._sync_btn.setFixedHeight(22)
        self._sync_btn.setStyleSheet("""
            QPushButton {
                background:#0e1018; color:#506070;
                border:1px solid #1a2030; border-radius:3px;
                font-size:8px; padding:2px 6px;
            }
            QPushButton:hover { color:#a0b8c8; border-color:#2a3040; }
        """)
        self._sync_btn.clicked.connect(self._sync_to_left)
        sync_row.addStretch()
        sync_row.addWidget(self._sync_btn)
        sync_row.addStretch()

        outer = QVBoxLayout()
        outer.setSpacing(4)
        inner_box = QWidget()
        inner_box.setLayout(root)
        outer.addWidget(inner_box)
        outer.addLayout(sync_row)
        self.setLayout(outer)

    def _on_left(self, val):
        v = val / 100.0
        self.left_val.setText(f"{v:.2f}")
        self.left_changed.emit(v)

    def _on_right(self, val):
        v = val / 100.0
        self.right_val.setText(f"{v:.2f}")
        self.right_changed.emit(v)

    def _sync_to_left(self):
        """Set right slider to match left."""
        self.right_slider.setValue(self.left_slider.value())

    def set_left(self, v: float):
        self.left_slider.blockSignals(True)
        self.left_slider.setValue(int(v * 100))
        self.left_val.setText(f"{v:.2f}")
        self.left_slider.blockSignals(False)

    def set_right(self, v: float):
        self.right_slider.blockSignals(True)
        self.right_slider.setValue(int(v * 100))
        self.right_val.setText(f"{v:.2f}")
        self.right_slider.blockSignals(False)


# ═════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ═════════════════════════════════════════════════════════════════════════
class MissionControl(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lunar Rover Mission Control")
        self.setMinimumWidth(900)
        self.setMinimumHeight(760)

        self._teleop_active = False
        self._teleop_thread: TeleopPublisher | None = None
        self._servo_angle = 90

        self._apply_stylesheet()
        self._build_ui()
        self._start_connection_checker()

    # ── Stylesheet ────────────────────────────────────────────────────────
    def _apply_stylesheet(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #0e1018;
                color: #c0cce0;
                font-family: 'Courier New', monospace;
            }
            QGroupBox {
                border: 1px solid #2a3040;
                border-radius: 6px;
                margin-top: 8px;
                font-size: 9px;
                font-weight: bold;
                color: #6080a0;
                letter-spacing: 1px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
            QTextEdit {
                background: #090b10; color: #7090a8;
                border: 1px solid #1a2030; border-radius:4px;
                font-family: monospace; font-size: 9px;
            }
            QLabel { color:#c0cce0; font-size:10px; }
        """)

    # ── UI ────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 12)

        # Header
        hdr = QLabel("⬡  LUNAR ROVER  ·  MISSION CONTROL")
        hdr.setStyleSheet("color:#e8a030; font-size:14px; font-weight:bold; "
                          "letter-spacing:3px; padding:4px 0;")
        root.addWidget(hdr)

        # Connection bar
        cbar = QHBoxLayout()
        self.conn_led   = StatusLED("off")
        self.conn_label = QLabel("miniPC: checking…")
        self.conn_label.setStyleSheet("color:#506070; font-size:9px;")
        cbar.addWidget(self.conn_led)
        cbar.addWidget(self.conn_label)
        cbar.addStretch()
        root.addLayout(cbar)

        # Two-column layout
        cols = QHBoxLayout()
        cols.setSpacing(10)
        left  = QVBoxLayout()
        right = QVBoxLayout()
        cols.addLayout(left,  50)
        cols.addLayout(right, 50)
        root.addLayout(cols)

        # ── LEFT: miniPC ──────────────────────────────────────────────────
        minipc_box = QGroupBox("MINI PC  ·  remote launch")
        ml = QVBoxLayout(minipc_box)
        delay_row = QHBoxLayout()
        delay_row.addWidget(QLabel("Delay (s):"))
        self.delay_input = QLineEdit("0")
        self.delay_input.setFixedWidth(50)
        self.delay_input.setStyleSheet(
            "background:#0e1018; color:#e8a030; border:1px solid #2a3040;"
            "border-radius:3px; padding:2px 4px;")
        delay_row.addWidget(self.delay_input)
        delay_row.addStretch()
        ml.addLayout(delay_row)
        self.minipc_led    = StatusLED("off")
        self.minipc_status = QLabel("not started")
        self.minipc_status.setStyleSheet("color:#506070; font-size:9px;")
        srow = QHBoxLayout()
        srow.addWidget(self.minipc_led)
        srow.addWidget(self.minipc_status)
        srow.addStretch()
        ml.addLayout(srow)
        self.minipc_btn = self._make_btn("LAUNCH MINI PC", "#1a1e10", "#4a6020", "#80aa30")
        self.minipc_btn.clicked.connect(self._launch_minipc)
        ml.addWidget(self.minipc_btn)
        left.addWidget(minipc_box)

        # ── LEFT: RViz ────────────────────────────────────────────────────
        rviz_card = ProcessCard("VISUALIZATION  ·  RViz2", self._rviz_cmd)
        left.addWidget(rviz_card)

        # ── LEFT: Autonomy ────────────────────────────────────────────────
        auto_box = QGroupBox("AUTONOMY")
        al = QVBoxLayout(auto_box)
        nav_btn  = self._make_btn("Point-Click Navigation", "#101820", "#1a4060", "#2a80c0")
        slam_btn = self._make_btn("SLAM / Mapping",         "#101820", "#1a4060", "#2a80c0")
        nav_btn.clicked.connect(self._start_nav)
        slam_btn.clicked.connect(self._start_slam)
        al.addWidget(nav_btn)
        al.addWidget(slam_btn)
        left.addWidget(auto_box)

        # ── LEFT: System log ──────────────────────────────────────────────
        log_box = QGroupBox("SYSTEM LOG")
        ll = QVBoxLayout(log_box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFixedHeight(110)
        ll.addWidget(self.log_view)
        left.addWidget(log_box)

        stop_all = self._make_btn("⬛  STOP ALL PROCESSES", "#1a0808", "#601010", "#aa2020")
        stop_all.clicked.connect(self._stop_all)
        left.addWidget(stop_all)
        left.addStretch()

        # ── RIGHT: Teleop ─────────────────────────────────────────────────
        tbox = QGroupBox("TELEOP  ·  TANK DRIVE  ·  /cmd_vel via gamepad")
        tl = QVBoxLayout(tbox)
        tl.setSpacing(6)

        note = QLabel("🎮  Left stick Y = LEFT wheels  ·  Right stick Y = RIGHT wheels")
        note.setStyleSheet("color:#3a8a50; font-size:9px; padding:2px 0;")
        tl.addWidget(note)

        # Start / E-stop row
        tr = QHBoxLayout()
        self.teleop_led = StatusLED("off")
        tr.addWidget(self.teleop_led)
        self.teleop_btn = self._make_btn("START TELEOP", "#1d3020", "#3a7a40", "#50aa60")
        self.teleop_btn.setCheckable(True)
        self.teleop_btn.clicked.connect(self._toggle_teleop)
        tr.addWidget(self.teleop_btn)
        self.estop_btn = self._make_btn("E-STOP", "#300d0d", "#a02020", "#ff4040")
        self.estop_btn.clicked.connect(self._emergency_stop)
        tr.addWidget(self.estop_btn)
        tl.addLayout(tr)

        ctrl_info = QLabel(
            "Y/A = right spd+/−  ·  X/B = left spd+/−\n"
            "LB = extend  ·  RB = retract  ·  Start = e-stop"
        )
        ctrl_info.setStyleSheet("color:#3a5060; font-size:9px; padding:2px 0;")
        tl.addWidget(ctrl_info)

        # ── Dual speed sliders ────────────────────────────────────────────
        self.dual_speed = DualSpeedWidget()
        self.dual_speed.left_changed.connect(self._left_speed_changed)
        self.dual_speed.right_changed.connect(self._right_speed_changed)
        tl.addWidget(self.dual_speed)

        right.addWidget(tbox)

        # ── RIGHT: Servo ──────────────────────────────────────────────────
        servo_box = QGroupBox("SERVO  ·  position control")
        sl = QVBoxLayout(servo_box)
        sl.setSpacing(6)

        servo_hint = QLabel("D-pad: UP = 90°  ·  LEFT = 0°  ·  RIGHT = 180°")
        servo_hint.setStyleSheet("color:#405060; font-size:9px;")
        sl.addWidget(servo_hint)

        # Gauge
        self.servo_gauge = ServoGauge()
        sl.addWidget(self.servo_gauge)

        # Manual position slider
        sangle_row = QHBoxLayout()
        sangle_lbl = QLabel("Manual:")
        sangle_lbl.setStyleSheet("color:#506070; font-size:9px;")
        sangle_row.addWidget(sangle_lbl)

        self.servo_slider = QSlider(Qt.Horizontal)
        self.servo_slider.setRange(0, 180)
        self.servo_slider.setValue(90)
        self.servo_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                background:#1a1e28; height:6px; border-radius:3px;
            }
            QSlider::handle:horizontal {
                background:#e8a030; width:14px; height:14px;
                margin:-4px 0; border-radius:7px;
            }
            QSlider::sub-page:horizontal { background:#503010; border-radius:3px; }
        """)
        self.servo_slider.valueChanged.connect(self._servo_slider_changed)
        sangle_row.addWidget(self.servo_slider)
        self.servo_angle_label = QLabel("90°")
        self.servo_angle_label.setStyleSheet(
            "color:#e8a030; font-size:11px; font-weight:bold; min-width:38px;")
        sangle_row.addWidget(self.servo_angle_label)
        sl.addLayout(sangle_row)

        # Preset buttons row
        preset_row = QHBoxLayout()
        preset_row.setSpacing(6)
        for label, angle, col in [
            ("0°  ◀", 0,   "#2a80c0"),
            ("90° ▲", 90,  "#50aa60"),
            ("180° ▶", 180, "#e8a030"),
        ]:
            btn = QPushButton(label)
            btn.setFixedHeight(26)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background:#0e1018; color:{col};
                    border:1px solid {col}66; border-radius:4px;
                    font-size:9px; font-weight:bold; padding:2px 8px;
                }}
                QPushButton:hover {{ background:{col}22; }}
                QPushButton:pressed {{ background:{col}44; }}
            """)
            btn.clicked.connect(lambda _, a=angle: self._set_servo(a))
            preset_row.addWidget(btn)
        sl.addLayout(preset_row)

        right.addWidget(servo_box)

        # ── RIGHT: Actuators ──────────────────────────────────────────────
        abox = QGroupBox("ACTUATORS  ·  /actuator_cmd")
        al2 = QVBoxLayout(abox)
        al2.setSpacing(6)

        ah = QLabel("LB = Extend  ·  RB = Retract  (hold)")
        ah.setStyleSheet("color:#405060; font-size:9px;")
        al2.addWidget(ah)

        abr = QHBoxLayout()
        self.act_extend_btn  = self._make_btn("▲  EXTEND",  "#1a2820", "#2a6040", "#40c070")
        self.act_retract_btn = self._make_btn("▼  RETRACT", "#281a1a", "#602a2a", "#c04040")
        self.act_extend_btn.pressed.connect(lambda:  self._act_gui(1))
        self.act_extend_btn.released.connect(lambda: self._act_gui(0))
        self.act_retract_btn.pressed.connect(lambda:  self._act_gui(-1))
        self.act_retract_btn.released.connect(lambda: self._act_gui(0))
        abr.addWidget(self.act_extend_btn)
        abr.addWidget(self.act_retract_btn)
        al2.addLayout(abr)

        self.act_status = QLabel("Actuator: idle")
        self.act_status.setStyleSheet("color:#607080; font-size:9px;")
        al2.addWidget(self.act_status)
        right.addWidget(abox)

        right.addStretch()
        root.addStretch()

        self._log("Mission Control ready  ·  TANK DRIVE mode")
        self._log("Left stick Y = left wheels  |  Right stick Y = right wheels")
        self._log("Y/A = right speed  ·  X/B = left speed  ·  D-pad = servo")
        if not ROS_AVAILABLE:
            self._log("⚠  rclpy not found — teleop disabled")

    # ── Button factory ────────────────────────────────────────────────────
    @staticmethod
    def _make_btn(text, bg, border, hover):
        btn = QPushButton(text)
        btn.setStyleSheet(f"""
            QPushButton {{
                background:{bg}; color:#a0b8c8;
                border:1px solid {border}; border-radius:5px;
                padding:6px 12px; font-size:10px; font-weight:bold;
                letter-spacing:1px;
            }}
            QPushButton:hover   {{ background:{border}; color:white; }}
            QPushButton:pressed {{ background:{bg}; }}
            QPushButton:checked {{ background:{border}; color:white; }}
        """)
        return btn

    # ── Logging ───────────────────────────────────────────────────────────
    def _log(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.append(
            f"<span style='color:#304050'>[{ts}]</span> {msg}")

    # ── Connection checker ────────────────────────────────────────────────
    def _start_connection_checker(self):
        self._conn_timer = QTimer()
        self._conn_timer.timeout.connect(self._check_connection)
        self._conn_timer.start(6000)
        self._check_connection()

    def _check_connection(self):
        def run():
            import subprocess as sp
            r = sp.run(f"ping -c1 -W2 {MINIPC_IP}", shell=True,
                       capture_output=True)
            ok = r.returncode == 0
            self.conn_led.set_color("green" if ok else "red")
            self.conn_label.setText(
                f"miniPC {MINIPC_IP}: {'online' if ok else 'offline'}")
        threading.Thread(target=run, daemon=True).start()

    # ── MiniPC launch ─────────────────────────────────────────────────────
    def _launch_minipc(self):
        try:
            delay = float(self.delay_input.text() or "0")
        except ValueError:
            delay = 0.0
        delay_str = f"{delay:.1f}"
        mode = f"competition delay {delay_str}s" if delay > 0 else "live mode"
        self._log(f"SSH-starting miniPC… ({mode})")
        self.minipc_led.set_color("yellow")
        self.minipc_status.setText("Starting…")
        cmd = (
            f'ssh -o ConnectTimeout=6 {MINIPC_USER}@{MINIPC_IP} '
            f'"DELAY_SEC={delay_str} nohup bash {MINIPC_WS}/full_launch_minipc.sh '
            f'> /tmp/minipc_launch.log 2>&1 &"'
        )
        def run():
            r = subprocess.run(cmd, shell=True)
            if r.returncode == 0:
                self._log("MiniPC launch sent ✓")
                self.minipc_led.set_color("green")
                self.minipc_status.setText(f"Launched  ·  {mode}")
            else:
                self._log("SSH failed — is miniPC reachable?")
                self.minipc_led.set_color("red")
                self.minipc_status.setText("SSH failed")
        threading.Thread(target=run, daemon=True).start()

    # ── RViz ─────────────────────────────────────────────────────────────
    def _rviz_cmd(self):
        if os.path.exists(RVIZ_CONFIG):
            return f"rviz2 -d {RVIZ_CONFIG}"
        return "rviz2"

    # ── Autonomy ─────────────────────────────────────────────────────────
    def _start_nav(self):
        self._log("Launching point-click navigation…")
        subprocess.Popen(
            "bash -c 'source /opt/ros/$(ls /opt/ros)/setup.bash && "
            "source ~/lunar_rover_ws/install/setup.bash 2>/dev/null; "
            "ros2 launch lunar_robot_hardware arduino_navigation.launch.py'",
            shell=True
        )

    def _start_slam(self):
        self._log("Launching SLAM…")
        subprocess.Popen(
            f'ssh {MINIPC_USER}@{MINIPC_IP} "bash {MINIPC_WS}/slam_minipc.sh map"',
            shell=True
        )

    # ── Teleop ────────────────────────────────────────────────────────────
    def _toggle_teleop(self, checked):
        if checked: self._start_teleop()
        else:       self._stop_teleop()

    def _start_teleop(self):
        if self._teleop_thread and self._teleop_thread.isRunning():
            return
        self._teleop_thread = TeleopPublisher()
        self._teleop_thread.status_changed.connect(self._log)
        self._teleop_thread.speed_left_changed.connect(self._on_ctrl_speed_left)
        self._teleop_thread.speed_right_changed.connect(self._on_ctrl_speed_right)
        self._teleop_thread.servo_changed.connect(self._on_ctrl_servo)
        # Sync current slider values into thread
        self._teleop_thread.set_speed_left(self.dual_speed.left_slider.value() / 100.0)
        self._teleop_thread.set_speed_right(self.dual_speed.right_slider.value() / 100.0)
        self._teleop_thread.start()
        self._teleop_active = True
        self.teleop_led.set_color("green")
        self.teleop_btn.setText("STOP TELEOP")
        self._log("Teleop started — Left stick=left wheels  Right stick=right wheels")
        self._log("Y/A=right spd  X/B=left spd  D-pad=servo  LB/RB=actuator")

    def _stop_teleop(self):
        if self._teleop_thread:
            self._teleop_thread.stop()
            self._teleop_thread = None
        self._teleop_active = False
        self.teleop_led.set_color("off")
        self.teleop_btn.setText("START TELEOP")
        self.teleop_btn.setChecked(False)
        self._log("Teleop stopped")

    def _emergency_stop(self):
        if self._teleop_thread:
            self._teleop_thread.emergency_stop()
        self._log("⬛ E-STOP")

    # ── Speed callbacks ───────────────────────────────────────────────────

    def _on_ctrl_speed_left(self, spd: float):
        """Controller button changed left speed → update GUI slider."""
        self.dual_speed.set_left(spd)

    def _on_ctrl_speed_right(self, spd: float):
        """Controller button changed right speed → update GUI slider."""
        self.dual_speed.set_right(spd)

    def _left_speed_changed(self, val: float):
        if self._teleop_thread:
            self._teleop_thread.set_speed_left(val)

    def _right_speed_changed(self, val: float):
        if self._teleop_thread:
            self._teleop_thread.set_speed_right(val)

    # ── Servo ─────────────────────────────────────────────────────────────

    def _set_servo(self, angle: int):
        """Set servo from GUI preset buttons or slider."""
        self._servo_angle = angle
        self.servo_gauge.set_angle(angle)
        self.servo_slider.blockSignals(True)
        self.servo_slider.setValue(angle)
        self.servo_slider.blockSignals(False)
        self.servo_angle_label.setText(f"{angle}°")
        self._log(f"Servo → {angle}°")

    def _servo_slider_changed(self, val: int):
        self._servo_angle = val
        self.servo_gauge.set_angle(val)
        self.servo_angle_label.setText(f"{val}°")

    def _on_ctrl_servo(self, angle: int):
        """Controller D-pad changed servo → update GUI."""
        self._set_servo(angle)

    # ── Actuator GUI ──────────────────────────────────────────────────────
    def _act_gui(self, value: int):
        if self._teleop_thread and hasattr(self._teleop_thread, '_want_act'):
            self._teleop_thread._want_act = value
        labels = {1: "▲ Extending…", -1: "▼ Retracting…", 0: "Actuator: idle"}
        colors = {1: "#40c070",       -1: "#c04040",        0: "#607080"}
        self.act_status.setText(labels.get(value, ""))
        self.act_status.setStyleSheet(
            f"color:{colors.get(value,'#607080')}; font-size:9px;")

    # ── Stop all ─────────────────────────────────────────────────────────
    def _stop_all(self):
        self._stop_teleop()
        self._log("Stopping all processes…")
        subprocess.run(
            "pkill -f rviz2; pkill -f unified_navigator; pkill -f slam_minipc",
            shell=True)


def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("Courier New", 10))
    w = MissionControl()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()