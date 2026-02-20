#!/usr/bin/env python3
"""
Lunar Rover Mission Control GUI
rover_control_gui.py

Key feature: Teleop publishes /cmd_vel LOCALLY on the laptop via a
rclpy node running in a QThread.  Messages travel to the miniPC over
DDS/UDP (~2 ms), completely eliminating SSH keystroke lag/stutter.
"""

import os
import sys
import math
import queue
import subprocess
import threading
import time

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit, QSlider,
    QSizePolicy, QFrame, QLineEdit
)
from PyQt5.QtGui import (
    QFont, QColor, QPainter, QBrush, QPen,
    QLinearGradient, QFontDatabase
)
from PyQt5.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, QPoint, QRect, QSize
)

# ── ROS2 optional import ─────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String as RosString, Int8 as RosInt8
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ── CONFIG ────────────────────────────────────────────────────────────────
MINIPC_USER = "moonpie"
MINIPC_IP   = "192.168.0.102"
MINIPC_WS   = "~/lunar_rover_ws"

RVIZ_CONFIG  = os.path.expanduser("~/lunar_rover_ws/laptop_stream.rviz")
PUBLISH_RATE = 20  # Hz for /cmd_vel

# ═════════════════════════════════════════════════════════════════════════
# TELEOP PUBLISHER — runs rclpy in a background QThread
# Publishing /cmd_vel LOCALLY means ROS DDS transports the message;
# no SSH round-trips, no key-up detection issues.
# ═════════════════════════════════════════════════════════════════════════
class TeleopPublisher(QThread):
    status_changed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._lock = threading.Lock()
        self._running = False
        self._linear  = 0.0
        self._angular  = 0.0
        self._speed    = 0.5   # m/s scale
        self._node     = None
        self._pub      = None

        # Keyboard state
        self._keys: set = set()
        self._actuator_pub  = None   # publishes /actuator_cmd
        self._actuator_queue = queue.Queue()  # thread-safe command delivery

    def set_speed(self, v: float):
        with self._lock:
            self._speed = max(0.05, min(1.0, v))

    def key_press(self, key: Qt.Key):
        self._keys.add(key)

    def key_release(self, key: Qt.Key):
        self._keys.discard(key)

    def emergency_stop(self):
        self._keys.clear()
        with self._lock:
            self._linear = 0.0
            self._angular = 0.0

    def set_velocity(self, linear: float, angular: float):
        """Direct velocity set (used by virtual joystick)."""
        with self._lock:
            self._linear  = linear
            self._angular = angular

    def _compute_from_keys(self):
        lin = ang = 0.0
        spd = self._speed
        with self._lock:
            s = self._speed
        if Qt.Key_W in self._keys: ang  =  s * 1.5   # W → forward (was linear, now angular)
        if Qt.Key_S in self._keys: ang  = -s * 1.5   # S → backward
        if Qt.Key_A in self._keys: lin  =  s          # A → left (was angular, now linear)
        if Qt.Key_D in self._keys: lin  = -s          # D → right
        with self._lock:
            self._linear  = lin
            self._angular = ang

    def run(self):
        if not ROS_AVAILABLE:
            self.status_changed.emit("ROS2 not available — install rclpy")
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node('rover_laptop_teleop')
            self._pub  = self._node.create_publisher(Twist, '/cmd_vel', 10)
            self._actuator_pub = self._node.create_publisher(RosInt8, '/actuator_cmd', 10)
            self._running = True
            self.status_changed.emit("Teleop node running")

            period = 1.0 / PUBLISH_RATE
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._node)

            next_time = time.monotonic()
            while self._running and rclpy.ok():
                self._compute_from_keys()

                # Publish cmd_vel
                msg = Twist()
                with self._lock:
                    msg.linear.x  = float(self._linear)
                    msg.angular.z = float(self._angular)
                self._pub.publish(msg)

                # Drain actuator queue — all pending commands published from ROS thread
                while True:
                    try:
                        act_cmd = self._actuator_queue.get_nowait()
                        act_msg = RosInt8()
                        # extend=+1, retract=-1, stop=0  (matches arduino_motor_controller.py)
                        act_msg.data = {"extend": 1, "retract": -1, "stop": 0}.get(act_cmd, 0)
                        self._actuator_pub.publish(act_msg)
                    except queue.Empty:
                        break

                executor.spin_once(timeout_sec=0.0)

                # Accurate sleep: compensate for processing time to prevent drift/stutter
                next_time += period
                sleep_for = next_time - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_time = time.monotonic()  # fallen behind; reset without pile-up

        except Exception as e:
            self.status_changed.emit(f"Teleop error: {e}")
        finally:
            self._running = False
            if self._node:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass

    def send_actuator(self, command: str):
        """Queue an actuator command for the ROS thread to publish safely."""
        self._actuator_queue.put(command)

    def stop(self):
        self._running = False
        self.emergency_stop()
        self.quit()
        self.wait(2000)


# ═════════════════════════════════════════════════════════════════════════
# VIRTUAL JOYSTICK WIDGET
# ═════════════════════════════════════════════════════════════════════════
class JoystickWidget(QWidget):
    velocity_changed = pyqtSignal(float, float)   # linear, angular

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(140, 140)
        self._center = QPoint(70, 70)
        self._stick  = QPoint(70, 70)
        self._radius = 55
        self._dragging = False

    def _emit(self):
        dx = (self._stick.x() - self._center.x()) / self._radius
        dy = (self._stick.y() - self._center.y()) / self._radius
        linear  = -dx  # left/right → linear.x (hardware swap)
        angular = -dy  # up/down   → angular.z
        self.velocity_changed.emit(
            max(-1.0, min(1.0, linear)),
            max(-1.0, min(1.0, angular))
        )

    def mousePressEvent(self, e):
        self._dragging = True
        self._update_stick(e.pos())

    def mouseMoveEvent(self, e):
        if self._dragging:
            self._update_stick(e.pos())

    def mouseReleaseEvent(self, e):
        self._dragging = False
        self._stick = QPoint(self._center)
        self._emit()
        self.update()

    def _update_stick(self, pos):
        dx = pos.x() - self._center.x()
        dy = pos.y() - self._center.y()
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > self._radius:
            dx = dx / dist * self._radius
            dy = dy / dist * self._radius
        self._stick = QPoint(int(self._center.x() + dx),
                             int(self._center.y() + dy))
        self._emit()
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # Background
        p.setBrush(QBrush(QColor(20, 22, 30)))
        p.setPen(QPen(QColor(60, 65, 80), 2))
        p.drawEllipse(self._center, self._radius + 4, self._radius + 4)

        # Guide crosshairs
        p.setPen(QPen(QColor(50, 55, 70), 1))
        p.drawLine(self._center.x() - self._radius, self._center.y(),
                   self._center.x() + self._radius, self._center.y())
        p.drawLine(self._center.x(), self._center.y() - self._radius,
                   self._center.x(), self._center.y() + self._radius)

        # Stick
        grad = QLinearGradient(self._stick.x()-15, self._stick.y()-15,
                               self._stick.x()+15, self._stick.y()+15)
        grad.setColorAt(0, QColor(255, 140, 40))
        grad.setColorAt(1, QColor(200, 80, 10))
        p.setBrush(QBrush(grad))
        p.setPen(QPen(QColor(255, 180, 80), 1))
        p.drawEllipse(self._stick, 18, 18)


# ═════════════════════════════════════════════════════════════════════════
# STATUS LED
# ═════════════════════════════════════════════════════════════════════════
class StatusLED(QLabel):
    COLORS = {
        "off":     QColor(40, 42, 50),
        "green":   QColor(60, 220, 80),
        "yellow":  QColor(255, 200, 40),
        "red":     QColor(220, 50, 50),
        "blue":    QColor(60, 140, 255),
    }

    def __init__(self, color="off"):
        super().__init__()
        self._color = color
        self.setFixedSize(14, 14)

    def set_color(self, c: str):
        self._color = c
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = self.COLORS.get(self._color, self.COLORS["off"])
        p.setBrush(QBrush(c))
        p.setPen(QPen(c.darker(150), 1))
        p.drawEllipse(1, 1, 12, 12)


# ═════════════════════════════════════════════════════════════════════════
# PROCESS CARD — unified start/stop/log widget for each service
# ═════════════════════════════════════════════════════════════════════════
class ProcessCard(QGroupBox):
    def __init__(self, title: str, command_fn, parent=None):
        super().__init__(title, parent)
        self._command_fn = command_fn
        self._proc: subprocess.Popen | None = None

        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid #2a2d3a;
                border-radius: 6px;
                margin-top: 8px;
                padding: 6px;
                color: #b0b8c8;
                font-size: 11px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                color: #8090aa;
            }
        """)

        row = QHBoxLayout(self)
        row.setContentsMargins(6, 12, 6, 6)

        self.led = StatusLED("off")
        row.addWidget(self.led)

        self.start_btn = QPushButton("START")
        self.start_btn.setFixedWidth(70)
        self.start_btn.clicked.connect(self._start)
        row.addWidget(self.start_btn)

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setFixedWidth(70)
        self.stop_btn.clicked.connect(self._stop)
        row.addWidget(self.stop_btn)

        self.status_label = QLabel("Idle")
        self.status_label.setStyleSheet("color: #606880; font-size: 10px;")
        row.addWidget(self.status_label)

        for btn in (self.start_btn, self.stop_btn):
            btn.setStyleSheet("""
                QPushButton {
                    background: #1e2130;
                    color: #8090aa;
                    border: 1px solid #2a2d3a;
                    border-radius: 4px;
                    padding: 4px 8px;
                    font-size: 10px;
                    font-weight: bold;
                }
                QPushButton:hover { background: #252840; color: #c0d0e8; }
                QPushButton:pressed { background: #1a1c28; }
            """)

        # Poll process state
        self._timer = QTimer()
        self._timer.timeout.connect(self._check)
        self._timer.start(800)

    def _start(self):
        if self._proc and self._proc.poll() is None:
            return
        cmd = self._command_fn()
        if not cmd:
            return
        self._proc = subprocess.Popen(
            cmd, shell=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        self.led.set_color("yellow")
        self.status_label.setText("Starting…")

    def _stop(self):
        if self._proc:
            self._proc.terminate()
            self._proc = None
        self.led.set_color("off")
        self.status_label.setText("Stopped")

    def _check(self):
        if self._proc:
            if self._proc.poll() is None:
                self.led.set_color("green")
                self.status_label.setText("Running")
            else:
                self.led.set_color("red")
                self.status_label.setText("Exited")
        else:
            self.led.set_color("off")


# ═════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ═════════════════════════════════════════════════════════════════════════
class MissionControl(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lunar Rover Mission Control")
        self.setMinimumWidth(820)
        self.setMinimumHeight(700)

        self._teleop_active = False
        self._teleop_thread: TeleopPublisher | None = None

        # Track held keys for WASD
        self._held: set = set()

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
            QLabel#header {
                color: #e8a030;
                font-size: 22px;
                font-weight: bold;
                letter-spacing: 3px;
            }
            QLabel#sub {
                color: #506070;
                font-size: 10px;
                letter-spacing: 2px;
            }
            QGroupBox {
                border: 1px solid #1e2230;
                border-radius: 6px;
                margin-top: 10px;
                padding: 8px;
                color: #708090;
                font-size: 11px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                color: #607080;
            }
            QTextEdit {
                background: #090b10;
                color: #50c878;
                border: 1px solid #1a1e28;
                border-radius: 4px;
                font-family: 'Courier New', monospace;
                font-size: 10px;
            }
            QSlider::groove:horizontal {
                height: 4px;
                background: #1a1e28;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: #e8a030;
                width: 14px;
                height: 14px;
                margin: -5px 0;
                border-radius: 7px;
            }
            QSlider::sub-page:horizontal {
                background: #c07820;
                border-radius: 2px;
            }
        """)

    # ── Build UI ──────────────────────────────────────────────────────────
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(10)

        # Header
        hdr_row = QHBoxLayout()
        title = QLabel("LUNAR ROVER  ·  MISSION CONTROL")
        title.setObjectName("header")
        hdr_row.addWidget(title)
        hdr_row.addStretch()
        self.conn_led = StatusLED("off")
        hdr_row.addWidget(self.conn_led)
        self.conn_label = QLabel("  Checking connection…")
        self.conn_label.setObjectName("sub")
        hdr_row.addWidget(self.conn_label)
        root.addLayout(hdr_row)

        div = QFrame()
        div.setFixedHeight(1)
        div.setStyleSheet("background: #1e2230;")
        root.addWidget(div)

        # Two-column layout
        cols = QHBoxLayout()
        cols.setSpacing(10)
        left  = QVBoxLayout()
        right = QVBoxLayout()
        cols.addLayout(left,  55)
        cols.addLayout(right, 45)
        root.addLayout(cols)

        # ── LEFT COLUMN ───────────────────────────────────────────────────

        # 1. Mini PC
        minipc_box = QGroupBox("MINI PC  ·  rover brain")
        mb_layout = QVBoxLayout(minipc_box)

        launch_row = QHBoxLayout()
        self.minipc_led = StatusLED("off")
        launch_row.addWidget(self.minipc_led)
        self.minipc_start = self._make_btn("START MINI PC", "#1d3020", "#3a7a40", "#50aa60")
        self.minipc_start.clicked.connect(self._start_minipc)
        launch_row.addWidget(self.minipc_start)
        self.minipc_status = QLabel("Not started")
        self.minipc_status.setStyleSheet("color:#506070; font-size:10px;")
        launch_row.addWidget(self.minipc_status)
        mb_layout.addLayout(launch_row)

        # Competition delay row
        delay_row = QHBoxLayout()
        delay_lbl = QLabel("⏱ Competition delay:")
        delay_lbl.setStyleSheet("color:#8090aa; font-size:10px;")
        delay_row.addWidget(delay_lbl)
        self.delay_input = QLineEdit("0.0")
        self.delay_input.setFixedWidth(55)
        self.delay_input.setPlaceholderText("0.0")
        self.delay_input.setStyleSheet(
            "background:#1a1e28; color:#e8a030; border:1px solid #2a3040; "
            "border-radius:3px; padding:2px 4px; font-size:11px;"
        )
        delay_row.addWidget(self.delay_input)
        delay_sec_lbl = QLabel("sec  (0 = live mode)")
        delay_sec_lbl.setStyleSheet("color:#506070; font-size:10px;")
        delay_row.addWidget(delay_sec_lbl)
        delay_row.addStretch()
        mb_layout.addLayout(delay_row)

        left.addWidget(minipc_box)

        # 2. RViz
        rviz_card = ProcessCard("VISUALIZATION  ·  RViz2", self._rviz_cmd)
        left.addWidget(rviz_card)

        # 3. Autonomy
        auto_box = QGroupBox("AUTONOMY")
        auto_layout = QVBoxLayout(auto_box)
        auto_row = QHBoxLayout()

        self.nav_btn = self._make_btn("POINT-CLICK NAV", "#201828", "#6030a0", "#9060e0")
        self.nav_btn.clicked.connect(self._start_nav)
        auto_row.addWidget(self.nav_btn)

        self.slam_btn = self._make_btn("SLAM MAP", "#201828", "#6030a0", "#9060e0")
        self.slam_btn.clicked.connect(self._start_slam)
        auto_row.addWidget(self.slam_btn)

        auto_layout.addLayout(auto_row)
        left.addWidget(auto_box)

        # 4. System log
        log_box = QGroupBox("SYSTEM LOG")
        log_layout = QVBoxLayout(log_box)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(160)
        log_layout.addWidget(self.log)
        left.addWidget(log_box)

        # STOP ALL
        stop_all = self._make_btn("⬛  STOP ALL PROCESSES", "#300d0d", "#a02020", "#ff4040")
        stop_all.setMinimumHeight(40)
        stop_all.clicked.connect(self._stop_all)
        left.addWidget(stop_all)

        # ── RIGHT COLUMN — TELEOP ─────────────────────────────────────────
        tbox = QGroupBox("TELEOP  ·  Local /cmd_vel publisher")
        tlayout = QVBoxLayout(tbox)
        tlayout.setSpacing(8)

        # Lag-fix notice
        note = QLabel("⚡ Zero-lag: cmd_vel published locally via DDS")
        note.setStyleSheet("color:#3a8a50; font-size:9px; padding: 2px 0;")
        tlayout.addWidget(note)

        # START/STOP teleop
        teleop_row = QHBoxLayout()
        self.teleop_led = StatusLED("off")
        teleop_row.addWidget(self.teleop_led)
        self.teleop_btn = self._make_btn("START TELEOP", "#1d3020", "#3a7a40", "#50aa60")
        self.teleop_btn.setCheckable(True)
        self.teleop_btn.clicked.connect(self._toggle_teleop)
        teleop_row.addWidget(self.teleop_btn)
        self.estop_btn = self._make_btn("E-STOP", "#300d0d", "#a02020", "#ff4040")
        self.estop_btn.clicked.connect(self._emergency_stop)
        teleop_row.addWidget(self.estop_btn)
        tlayout.addLayout(teleop_row)

        # Keyboard hint
        keys = QLabel("WASD to drive  ·  focus this panel then press keys")
        keys.setStyleSheet("color:#405060; font-size:9px;")
        tlayout.addWidget(keys)

        # Velocity display
        vel_row = QHBoxLayout()
        vel_row.addWidget(QLabel("Linear:"))
        self.vel_lin = QLabel("0.00 m/s")
        self.vel_lin.setStyleSheet("color:#e8a030; font-weight:bold; min-width:80px;")
        vel_row.addWidget(self.vel_lin)
        vel_row.addWidget(QLabel("Angular:"))
        self.vel_ang = QLabel("0.00 r/s")
        self.vel_ang.setStyleSheet("color:#e8a030; font-weight:bold; min-width:80px;")
        vel_row.addWidget(self.vel_ang)
        tlayout.addLayout(vel_row)

        # Speed slider
        spd_row = QHBoxLayout()
        spd_row.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(5, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.valueChanged.connect(self._speed_changed)
        spd_row.addWidget(self.speed_slider)
        self.speed_label = QLabel("0.50 m/s")
        self.speed_label.setStyleSheet("color:#e8a030; min-width:55px;")
        spd_row.addWidget(self.speed_label)
        tlayout.addLayout(spd_row)

        # Joystick + WASD diagram
        joy_row = QHBoxLayout()
        joy_row.addStretch()

        self.joystick = JoystickWidget()
        self.joystick.velocity_changed.connect(self._joystick_moved)
        joy_row.addWidget(self.joystick)

        joy_row.addSpacing(16)

        # WASD visual
        wasd = self._make_wasd_widget()
        joy_row.addWidget(wasd)

        joy_row.addStretch()
        tlayout.addLayout(joy_row)

        right.addWidget(tbox)

        # ── ACTUATOR CONTROL ──────────────────────────────────────────────
        abox = QGroupBox("ACTUATORS  ·  /actuator_cmd")
        alayout = QVBoxLayout(abox)
        alayout.setSpacing(6)

        akeys_hint = QLabel("P = Extend  ·  L = Retract  ·  (teleop must be active)")
        akeys_hint.setStyleSheet("color:#405060; font-size:9px;")
        alayout.addWidget(akeys_hint)

        abtn_row = QHBoxLayout()

        self.act_extend_btn = self._make_btn("▲  EXTEND  [P]", "#1a2820", "#2a6040", "#40c070")
        self.act_extend_btn.pressed.connect(lambda: self._actuator_cmd("extend"))
        self.act_extend_btn.released.connect(lambda: self._actuator_cmd("stop"))
        abtn_row.addWidget(self.act_extend_btn)

        self.act_retract_btn = self._make_btn("▼  RETRACT  [L]", "#281a1a", "#602a2a", "#c04040")
        self.act_retract_btn.pressed.connect(lambda: self._actuator_cmd("retract"))
        self.act_retract_btn.released.connect(lambda: self._actuator_cmd("stop"))
        abtn_row.addWidget(self.act_retract_btn)

        alayout.addLayout(abtn_row)

        self.act_status = QLabel("Actuator: idle")
        self.act_status.setStyleSheet("color:#607080; font-size:9px;")
        alayout.addWidget(self.act_status)

        right.addWidget(abox)
        right.addStretch()

        root.addStretch()

        # ── Velocity update timer ─────────────────────────────────────────
        self._vel_timer = QTimer()
        self._vel_timer.timeout.connect(self._update_vel_display)
        self._vel_timer.start(100)

        self._log("Mission Control ready.")
        if not ROS_AVAILABLE:
            self._log("⚠ rclpy not found — teleop disabled (ROS2 not in PATH)")

    # ── WASD visual diagram ───────────────────────────────────────────────
    def _make_wasd_widget(self) -> QWidget:
        w = QWidget()
        w.setFixedSize(110, 110)
        layout = QVBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        def key_lbl(text):
            l = QLabel(text)
            l.setAlignment(Qt.AlignCenter)
            l.setFixedSize(30, 30)
            l.setStyleSheet("""
                background:#1a1e28; color:#8090aa;
                border:1px solid #2a2d3a; border-radius:4px;
                font-size:12px; font-weight:bold;
            """)
            return l

        r1 = QHBoxLayout(); r1.setSpacing(4)
        r1.addStretch()
        self._key_w = key_lbl("W"); r1.addWidget(self._key_w)
        r1.addStretch()
        layout.addLayout(r1)

        r2 = QHBoxLayout(); r2.setSpacing(4)
        r2.addStretch()
        self._key_a = key_lbl("A"); r2.addWidget(self._key_a)
        self._key_s = key_lbl("S"); r2.addWidget(self._key_s)
        self._key_d = key_lbl("D"); r2.addWidget(self._key_d)
        r2.addStretch()
        layout.addLayout(r2)

        hint = QLabel("SPACE = e-stop\nZ/X = speed ±")
        hint.setStyleSheet("color:#384858; font-size:8px;")
        hint.setAlignment(Qt.AlignCenter)
        layout.addWidget(hint)

        return w

    # ── Button factory ────────────────────────────────────────────────────
    @staticmethod
    def _make_btn(text, bg, border, hover):
        btn = QPushButton(text)
        btn.setStyleSheet(f"""
            QPushButton {{
                background: {bg};
                color: #a0b8c8;
                border: 1px solid {border};
                border-radius: 5px;
                padding: 6px 12px;
                font-size: 10px;
                font-weight: bold;
                letter-spacing: 1px;
            }}
            QPushButton:hover {{ background: {border}; color: white; }}
            QPushButton:pressed {{ background: {bg}; }}
            QPushButton:checked {{ background: {border}; color: white; }}
        """)
        return btn

    # ── Connection checker ────────────────────────────────────────────────
    def _start_connection_checker(self):
        self._conn_timer = QTimer()
        self._conn_timer.timeout.connect(self._check_connection)
        self._conn_timer.start(6000)
        QTimer.singleShot(500, self._check_connection)

    def _check_connection(self):
        def run():
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "2", MINIPC_IP],
                capture_output=True
            )
            reachable = result.returncode == 0

            if reachable:
                self.conn_led.set_color("green")
                self.conn_label.setText(f"  {MINIPC_USER}@{MINIPC_IP} reachable")
                self.minipc_led.set_color("green")
            else:
                self.conn_led.set_color("red")
                self.conn_label.setText(f"  {MINIPC_IP} unreachable")
                self.minipc_led.set_color("off")

        threading.Thread(target=run, daemon=True).start()

    # ── Mini PC ───────────────────────────────────────────────────────────
    def _start_minipc(self):
        # Read delay value from input box
        try:
            delay = float(self.delay_input.text().strip() or "0.0")
            delay = max(0.0, delay)
        except ValueError:
            delay = 0.0
            self.delay_input.setText("0.0")

        delay_str = f"{delay:.1f}"
        mode_str  = f"competition delay {delay_str}s" if delay > 0 else "live mode"
        self._log(f"SSH-starting mini PC at {MINIPC_USER}@{MINIPC_IP}… ({mode_str})")
        self.minipc_status.setText("Starting via SSH…")
        self.minipc_led.set_color("yellow")

        cmd = (
            f'ssh -o ConnectTimeout=6 {MINIPC_USER}@{MINIPC_IP} '
            f'"DELAY_SEC={delay_str} nohup bash {MINIPC_WS}/full_launch_minipc.sh '
            f'> /tmp/minipc_launch.log 2>&1 &"'
        )

        def run():
            result = subprocess.run(cmd, shell=True)
            if result.returncode == 0:
                self._log(f"Mini PC launch command sent ✓  (DELAY_SEC={delay_str})")
                self.minipc_status.setText(f"Launched  ·  {mode_str}")
                self.minipc_led.set_color("green")
            else:
                self._log("SSH failed — is miniPC reachable?")
                self.minipc_status.setText("SSH failed")
                self.minipc_led.set_color("red")

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
        self._log("Launching SLAM mini PC script…")
        cmd = (
            f'ssh {MINIPC_USER}@{MINIPC_IP} '
            f'"bash {MINIPC_WS}/slam_minipc.sh map"'
        )
        subprocess.Popen(cmd, shell=True)

    # ── Teleop ────────────────────────────────────────────────────────────
    def _toggle_teleop(self, checked):
        if checked:
            self._start_teleop()
        else:
            self._stop_teleop()

    def _start_teleop(self):
        if self._teleop_thread and self._teleop_thread.isRunning():
            return
        self._teleop_thread = TeleopPublisher()
        self._teleop_thread.status_changed.connect(self._log)
        self._teleop_thread.start()
        self._teleop_active = True
        self.teleop_led.set_color("green")
        self.teleop_btn.setText("STOP TELEOP")
        self._log("Teleop started — WASD keys or joystick, click here first")
        self.setFocus()

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
        self._log("⬛ E-STOP — all velocity zeroed")

    def _joystick_moved(self, linear: float, angular: float):
        if self._teleop_thread and self._teleop_active:
            spd = self.speed_slider.value() / 100.0
            self._teleop_thread.set_velocity(linear * spd, angular * spd * 1.5)

    def _speed_changed(self, val):
        spd = val / 100.0
        self.speed_label.setText(f"{spd:.2f} m/s")
        if self._teleop_thread:
            self._teleop_thread.set_speed(spd)

    # ── Key events (for WASD) ─────────────────────────────────────────────
    def keyPressEvent(self, e):
        if not self._teleop_active:
            return
        key = e.key()

        if key == Qt.Key_Space:
            self._emergency_stop()
            return
        if key == Qt.Key_Z:
            self.speed_slider.setValue(min(100, self.speed_slider.value() + 5))
            return
        if key == Qt.Key_X:
            self.speed_slider.setValue(max(5, self.speed_slider.value() - 5))
            return

        if key in (Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D):
            self._teleop_thread.key_press(key)
            self._update_key_visuals()

        if key == Qt.Key_P:
            self._actuator_cmd("extend")
        if key == Qt.Key_L:
            self._actuator_cmd("retract")

    def keyReleaseEvent(self, e):
        key = e.key()
        if key in (Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D) and self._teleop_thread:
            self._teleop_thread.key_release(key)
            self._update_key_visuals()
        if key in (Qt.Key_P, Qt.Key_L):
            self._actuator_cmd("stop")

    def _update_key_visuals(self):
        if not self._teleop_thread:
            return
        held = self._teleop_thread._keys
        active_style = "background:#e8a030; color:#000; border:1px solid #e8a030; border-radius:4px; font-size:12px; font-weight:bold;"
        inactive_style = "background:#1a1e28; color:#8090aa; border:1px solid #2a2d3a; border-radius:4px; font-size:12px; font-weight:bold;"
        for lbl, key in ((self._key_w, Qt.Key_W), (self._key_a, Qt.Key_A),
                          (self._key_s, Qt.Key_S), (self._key_d, Qt.Key_D)):
            lbl.setStyleSheet(active_style if key in held else inactive_style)

    def _actuator_cmd(self, command: str):
        """Send actuator command and update status label."""
        if self._teleop_thread:
            self._teleop_thread.send_actuator(command)
        label = {"extend": "▲ Extending…", "retract": "▼ Retracting…", "stop": "Actuator: idle"}.get(command, command)
        self.act_status.setText(label)
        color = {"extend": "#40c070", "retract": "#c04040", "stop": "#607080"}.get(command, "#607080")
        self.act_status.setStyleSheet(f"color:{color}; font-size:9px;")

    # ── Velocity display ──────────────────────────────────────────────────
    def _update_vel_display(self):
        if self._teleop_thread:
            with self._teleop_thread._lock:
                lin = self._teleop_thread._linear
                ang = self._teleop_thread._angular
            self.vel_lin.setText(f"{lin:+.2f} m/s")
            self.vel_ang.setText(f"{ang:+.2f} r/s")
        else:
            self.vel_lin.setText("0.00 m/s")
            self.vel_ang.setText("0.00 r/s")

    # ── Stop all ─────────────────────────────────────────────────────────
    def _stop_all(self):
        self._stop_teleop()
        self._log("Stopping all processes…")
        subprocess.run("pkill -f rviz2; pkill -f unified_navigator; "
                       "pkill -f slam_minipc", shell=True)
        self._log("Done")

    # ── Log helper ────────────────────────────────────────────────────────
    def _log(self, msg: str):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log.append(f"<span style='color:#384858'>[{ts}]</span> {msg}")

    # ── Cleanup ───────────────────────────────────────────────────────────
    def closeEvent(self, e):
        self._stop_teleop()
        e.accept()


# ═════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Lunar Rover Mission Control")

    window = MissionControl()
    window.show()
    window.setFocus()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()