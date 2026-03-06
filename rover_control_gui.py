#!/usr/bin/env python3
"""
Lunar Rover Mission Control GUI  —  rover_control_gui.py
Updated: corrected button mapping + live debug panel.
"""

import os, sys, math, subprocess, threading, time

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit, QSlider,
    QSizePolicy, QFrame, QLineEdit, QTabWidget
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

# ── Servo angles ──────────────────────────────────────────────────────────
SERVO_STOP = 90
SERVO_CCW  = 45
SERVO_CW   = 135

# ── Controller button/axis mapping  (must match joy_to_arduino.py) ────────
AXIS_LEFT        = 1
AXIS_RIGHT       = 4
AXIS_LT          = 2
AXIS_RT          = 5
TRIGGER_THRESHOLD = 0.5

BTN_LB    = 4   # LEFT  speed UP
BTN_RB    = 5   # RIGHT speed UP
BTN_A     = 0   # DUMP  position
BTN_Y     = 3   # DRIVE position
BTN_B     = 1   # DIG   position
BTN_X     = 2   # CALIBRATE
BTN_START = 7   # e-stop

DPAD_AXIS_LR = 6
DPAD_AXIS_UD = 7


# ═════════════════════════════════════════════════════════════════════════
# SERVO GAUGE WIDGET
# ═════════════════════════════════════════════════════════════════════════

class ServoGauge(QWidget):
    """Arc gauge showing servo position 0–180°."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._angle = 90
        self._target = 90
        self._anim_timer = QTimer()
        self._anim_timer.timeout.connect(self._step_anim)
        self.setMinimumSize(140, 90)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_angle(self, angle: int):
        self._target = max(0, min(180, angle))
        if not self._anim_timer.isActive():
            self._anim_timer.start(16)

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
        cx = w // 2
        cy = h - 10
        r_out = min(cx - 4, h - 14)
        r_in  = int(r_out * 0.60)

        arc_pen = QPen(QColor(30, 38, 52), 10)
        arc_pen.setCapStyle(Qt.RoundCap)
        p.setPen(arc_pen)
        p.drawArc(QRect(cx - r_out, cy - r_out, r_out * 2, r_out * 2),
                  0 * 16, 180 * 16)

        grad = QLinearGradient(cx - r_out, cy, cx + r_out, cy)
        grad.setColorAt(0.0, QColor(42, 128, 192))
        grad.setColorAt(0.5, QColor(80, 200, 160))
        grad.setColorAt(1.0, QColor(232, 160, 48))
        fill_pen = QPen(grad, 10)
        fill_pen.setCapStyle(Qt.RoundCap)
        p.setPen(fill_pen)
        span = int(self._angle)
        if span > 0:
            p.drawArc(QRect(cx - r_out, cy - r_out, r_out * 2, r_out * 2),
                      180 * 16, span * 16)

        tick_pen = QPen(QColor(50, 65, 85), 2)
        p.setPen(tick_pen)
        for deg in (0, 45, 90, 135, 180):
            rad = math.radians(180 - deg)
            x1 = cx + (r_out + 2) * math.cos(rad)
            y1 = cy - (r_out + 2) * math.sin(rad)
            x2 = cx + (r_out - 8) * math.cos(rad)
            y2 = cy - (r_out - 8) * math.sin(rad)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

        needle_rad = math.radians(180 - self._angle)
        nx = cx + r_in * math.cos(needle_rad)
        ny = cy - r_in * math.sin(needle_rad)
        needle_pen = QPen(QColor(232, 160, 48), 3)
        needle_pen.setCapStyle(Qt.RoundCap)
        p.setPen(needle_pen)
        p.drawLine(cx, cy, int(nx), int(ny))

        p.setBrush(QBrush(QColor(232, 160, 48)))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPoint(cx, cy), 5, 5)

        p.setPen(QPen(QColor(192, 204, 224)))
        font = QFont("Courier New", 9, QFont.Bold)
        p.setFont(font)
        p.drawText(QRect(0, 0, w, h - 2), Qt.AlignHCenter | Qt.AlignBottom,
                   f"{int(round(self._angle))}°")


# ═════════════════════════════════════════════════════════════════════════
# TELEOP PUBLISHER  (corrected button mapping)
# ═════════════════════════════════════════════════════════════════════════

class TeleopPublisher(QThread):
    status_changed      = pyqtSignal(str)
    speed_left_changed  = pyqtSignal(float)
    speed_right_changed = pyqtSignal(float)
    servo_changed       = pyqtSignal(int)
    # New signals for the debug panel
    joy_raw_signal      = pyqtSignal(str)   # raw axes/buttons string
    arduino_tx_signal   = pyqtSignal(str)   # every serial packet attempted

    SPEED_STEP = 0.05

    def __init__(self):
        super().__init__()
        self._lock         = threading.Lock()
        self._running      = False
        self._speed_left   = 0.5
        self._speed_right  = 0.5
        self._node         = None
        self._pub          = None
        self._act_pub      = None

        self._want_lin = 0.0
        self._want_ang = 0.0
        self._want_act = 0   # used only for /actuator_cmd relay

        self._sent_lin = None
        self._sent_ang = None
        self._sent_act = None

        self._prev_btns      = {}
        self._prev_dpad_lr   = 0.0
        self._prev_dpad_ud   = 0.0
        self._lt_was_pressed = False
        self._rt_was_pressed = False

        self._servo_moving = False

    # ── Helpers ───────────────────────────────────────────────────────────

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

    def _dz(self, v):
        return v if abs(v) >= 0.10 else 0.0

    def _rising(self, idx, cur):
        prev = self._prev_btns.get(idx, 0)
        self._prev_btns[idx] = cur
        return cur == 1 and prev == 0

    # ── /joy callback ─────────────────────────────────────────────────────

    def _joy_cb(self, msg):
        """
        GUI is DISPLAY ONLY — it does NOT handle buttons or actuators.
        All button/actuator/servo/speed commands are handled exclusively
        by joy_to_arduino.py on the miniPC.

        This callback only:
          1. Emits raw axis/button data for the debug panel
          2. Watches btn indices to show a human-readable event label
        """
        try:
            ax  = lambda i: msg.axes[i]    if i < len(msg.axes)    else 0.0
            btn = lambda i: msg.buttons[i] if i < len(msg.buttons) else 0

            # ── Emit raw data for the debug panel ─────────────────────────
            axes_str = ' '.join(f'{ax(i):+.2f}' for i in range(len(msg.axes)))
            btns_str = ' '.join(str(btn(i)) for i in range(len(msg.buttons)))
            self.joy_raw_signal.emit(f'axes=[{axes_str}]  btns=[{btns_str}]')

            # ── Human-readable event labels (display only, no commands) ───
            # BTN_LB / BTN_RB bumpers
            if self._rising(BTN_LB, btn(BTN_LB)):
                self.arduino_tx_signal.emit('[GUI sees] LB pressed → left spd+ (handled by miniPC)')
            if self._rising(BTN_RB, btn(BTN_RB)):
                self.arduino_tx_signal.emit('[GUI sees] RB pressed → right spd+ (handled by miniPC)')

            # Actuator buttons
            if self._rising(BTN_A, btn(BTN_A)):
                self.arduino_tx_signal.emit('[GUI sees] A pressed → DUMP cmd sent by miniPC')
                self.status_changed.emit('A → DUMP  (miniPC sending)')
            if self._rising(BTN_Y, btn(BTN_Y)):
                self.arduino_tx_signal.emit('[GUI sees] Y pressed → DRIVE cmd sent by miniPC')
                self.status_changed.emit('Y → DRIVE  (miniPC sending)')
            if self._rising(BTN_X, btn(BTN_X)):
                self.arduino_tx_signal.emit('[GUI sees] X pressed → CALIBRATE cmd sent by miniPC')
                self.status_changed.emit('X → CALIBRATE  (miniPC sending)')

            # D-pad servo (display feedback)
            cur_lr = ax(DPAD_AXIS_LR)
            if cur_lr > 0.5 and self._prev_dpad_lr <= 0.5:
                self.servo_changed.emit(SERVO_CW)
                self.arduino_tx_signal.emit('[GUI sees] D→ pressed → Servo CW (miniPC sending)')
            elif cur_lr < -0.5 and self._prev_dpad_lr >= -0.5:
                self.servo_changed.emit(SERVO_CCW)
                self.arduino_tx_signal.emit('[GUI sees] D← pressed → Servo CCW (miniPC sending)')
            elif abs(cur_lr) < 0.5 and abs(self._prev_dpad_lr) > 0.5:
                self.servo_changed.emit(SERVO_STOP)
                self.arduino_tx_signal.emit('[GUI sees] D-pad released → Servo STOP (miniPC sending)')
            self._prev_dpad_lr = cur_lr

            # LT / RT triggers (display only)
            lt_raw = ax(AXIS_LT)
            if lt_raw < (1.0 - TRIGGER_THRESHOLD) and not self._lt_was_pressed:
                self.arduino_tx_signal.emit('[GUI sees] LT pressed → left spd- (handled by miniPC)')
            self._lt_was_pressed = lt_raw < (1.0 - TRIGGER_THRESHOLD)

            rt_raw = ax(AXIS_RT)
            if rt_raw < (1.0 - TRIGGER_THRESHOLD) and not self._rt_was_pressed:
                self.arduino_tx_signal.emit('[GUI sees] RT pressed → right spd- (handled by miniPC)')
            self._rt_was_pressed = rt_raw < (1.0 - TRIGGER_THRESHOLD)

        except Exception as e:
            self.status_changed.emit(f'Joy display error: {e}')

    # ── Flush to ROS ──────────────────────────────────────────────────────

    def _flush(self):
        with self._lock:
            lin = self._want_lin
            ang = self._want_ang
            act = self._want_act

        if lin != self._sent_lin or ang != self._sent_ang:
            if self._pub:
                msg = Twist()
                msg.linear.x  = float(lin)
                msg.angular.z = float(ang)
                self._pub.publish(msg)
            self._sent_lin = lin
            self._sent_ang = ang

        if act != self._sent_act:
            if self._act_pub:
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
                self.status_changed.emit('Teleop active · controller connected')
            except Exception:
                self.status_changed.emit('Teleop active · no sensor_msgs')

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
    left_changed  = pyqtSignal(float)
    right_changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__("SPEED  ·  independent L / R", parent)
        self._build()

    def _build(self):
        root = QHBoxLayout(self)
        root.setSpacing(12)

        def make_side(label_text, color):
            col = QVBoxLayout()
            col.setSpacing(3)
            hdr = QLabel(label_text)
            hdr.setAlignment(Qt.AlignCenter)
            hdr.setStyleSheet(
                f"color:{color}; font-size:9px; font-weight:bold; letter-spacing:1px;")
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
                QSlider::sub-page:vertical {{ background:#1a1e28; }}
                QSlider::add-page:vertical {{
                    background:{color}44; border-radius:3px;
                }}
            """)
            val_lbl = QLabel("0.50")
            val_lbl.setAlignment(Qt.AlignCenter)
            val_lbl.setStyleSheet(
                f"color:{color}; font-size:11px; font-weight:bold;")
            slider_row = QHBoxLayout()
            slider_row.addStretch()
            slider_row.addWidget(slider)
            slider_row.addStretch()
            col.addLayout(slider_row)
            col.addWidget(val_lbl)
            return col, slider, val_lbl

        left_col,  self.left_slider,  self.left_val  = make_side(
            "◀  LEFT  (LB/LT)", "#50c878")
        right_col, self.right_slider, self.right_val = make_side(
            "RIGHT  ▶ (RB/RT)", "#e8a030")

        div = QFrame()
        div.setFrameShape(QFrame.VLine)
        div.setStyleSheet("color:#1a2030;")

        root.addLayout(left_col)
        root.addWidget(div)
        root.addLayout(right_col)

        self.left_slider.valueChanged.connect(self._on_left)
        self.right_slider.valueChanged.connect(self._on_right)

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
# DEBUG PANEL  (new)
# ═════════════════════════════════════════════════════════════════════════

class DebugPanel(QGroupBox):
    """
    Shows:
      • Live /joy axes + buttons (raw dump)
      • Every command event that joy_to_arduino would send
      • Live status from /joy_arduino_status topic
      • Encoder count + servo state
    """
    def __init__(self, parent=None):
        super().__init__("DEBUG  ·  live telemetry", parent)
        self._build()
        self._joy_count = 0
        self._last_joy_time = time.monotonic()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setSpacing(4)

        # Status row (serial / joy / enc)
        self._status_label = QLabel("Waiting for status…")
        self._status_label.setStyleSheet(
            "color:#50c8a0; font-size:9px; font-family:monospace;")
        self._status_label.setWordWrap(True)
        lay.addWidget(self._status_label)

        # Controller map reminder
        hint = QLabel(
            "  A=DUMP  Y=DRIVE  B=DIG  |  LB=L spd+  LT=L spd-  "
            "RB=R spd+  RT=R spd-  |  D← CCW  D→ CW  |  Start=estop")
        hint.setStyleSheet("color:#3a6060; font-size:8px;")
        hint.setWordWrap(True)
        lay.addWidget(hint)

        # Joy raw
        joy_hdr = QLabel("Controller raw (every ~1.5s):")
        joy_hdr.setStyleSheet("color:#506070; font-size:8px;")
        lay.addWidget(joy_hdr)

        self._joy_raw = QLabel("—  (no /joy messages yet)")
        self._joy_raw.setStyleSheet(
            "color:#4080c0; font-size:8px; font-family:monospace; "
            "background:#09111a; padding:2px 4px; border-radius:3px;")
        self._joy_raw.setWordWrap(True)
        lay.addWidget(self._joy_raw)

        # TX log
        tx_hdr = QLabel("Command events (last 12):")
        tx_hdr.setStyleSheet("color:#506070; font-size:8px;")
        lay.addWidget(tx_hdr)

        self._tx_log = QTextEdit()
        self._tx_log.setReadOnly(True)
        self._tx_log.setFixedHeight(130)
        self._tx_log.setStyleSheet(
            "background:#060e08; color:#40c060; "
            "border:1px solid #1a3020; border-radius:3px; "
            "font-family:monospace; font-size:8px;")
        lay.addWidget(self._tx_log)

        # Joy health indicator
        health_row = QHBoxLayout()
        self._joy_led = StatusLED("off")
        self._joy_health = QLabel("No /joy msgs")
        self._joy_health.setStyleSheet("color:#506070; font-size:9px;")
        health_row.addWidget(self._joy_led)
        health_row.addWidget(self._joy_health)
        health_row.addStretch()
        lay.addLayout(health_row)

        # Health check timer
        self._health_timer = QTimer()
        self._health_timer.timeout.connect(self._check_joy_health)
        self._health_timer.start(2000)

    def update_status(self, status_str: str):
        """Called when /joy_arduino_status arrives (parsed key=val pairs)."""
        parts = {}
        for item in status_str.split('|'):
            if '=' in item:
                k, v = item.split('=', 1)
                parts[k.strip()] = v.strip()

        serial_ok  = parts.get('serial', '?') == 'True'
        spd_l      = parts.get('spd_L',  '?')
        spd_r      = parts.get('spd_R',  '?')
        enc        = parts.get('enc',    '?')
        servo      = parts.get('servo',  '?')
        estop      = parts.get('estop',  '?')
        joy_total  = parts.get('joy_msgs', '?')

        color = '#50c8a0' if serial_ok else '#c85050'
        self._status_label.setStyleSheet(
            f"color:{color}; font-size:9px; font-family:monospace;")
        self._status_label.setText(
            f"serial={'OK' if serial_ok else 'NO PORT'}  "
            f"spd L={spd_l} R={spd_r}  "
            f"enc={enc}  servo={servo}  estop={estop}  "
            f"joy_total={joy_total}")

    def update_joy_raw(self, raw_str: str):
        self._joy_raw.setText(raw_str)
        self._joy_count += 1
        self._last_joy_time = time.monotonic()
        self._joy_led.set_color("green")
        self._joy_health.setText(f"Joy OK  ({self._joy_count} msgs)")
        self._joy_health.setStyleSheet("color:#50c870; font-size:9px;")

    def log_tx(self, msg: str):
        from datetime import datetime
        ts  = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        cur = self._tx_log.toPlainText().splitlines()
        lines = (cur + [f"[{ts}] {msg}"])[-12:]
        self._tx_log.setPlainText('\n'.join(lines))
        sb = self._tx_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _check_joy_health(self):
        age = time.monotonic() - self._last_joy_time
        if age > 3.0:
            self._joy_led.set_color("red")
            self._joy_health.setText(f"No /joy for {age:.0f}s ⚠")
            self._joy_health.setStyleSheet("color:#c05050; font-size:9px;")


# ═════════════════════════════════════════════════════════════════════════
# JOY STATUS SUBSCRIBER  (background ROS thread)
# ═════════════════════════════════════════════════════════════════════════

class JoyStatusSubscriber(QThread):
    status_received = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = False

    def run(self):
        if not ROS_AVAILABLE:
            return
        try:
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node('rover_gui_status_sub')
            node.create_subscription(
                String, '/joy_arduino_status',
                lambda msg: self.status_received.emit(msg.data),
                10)
            self._running = True
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception:
            pass

    def stop(self):
        self._running = False
        self.quit()
        self.wait(2000)


# ═════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ═════════════════════════════════════════════════════════════════════════

class MissionControl(QWidget):

    log_signal = pyqtSignal(str)   # thread-safe logging

    def __init__(self):
        super().__init__()
        self.log_signal.connect(self._log_direct)
        self.setWindowTitle("Lunar Rover Mission Control")
        self.setMinimumWidth(1000)
        self.setMinimumHeight(820)

        self._teleop_active  = False
        self._teleop_thread  = None
        self._status_thread  = None
        self._servo_angle    = 90

        self._apply_stylesheet()
        self._build_ui()
        self._start_connection_checker()
        self._start_status_subscriber()

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
            QTabWidget::pane { border:1px solid #2a3040; }
            QTabBar::tab {
                background:#0e1018; color:#506070;
                border:1px solid #2a3040; border-bottom:none;
                padding:4px 12px; font-size:9px;
            }
            QTabBar::tab:selected { background:#141820; color:#c0cce0; }
        """)

    # ── UI ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 12)

        # Header
        hdr = QLabel("⬡  LUNAR ROVER  ·  MISSION CONTROL")
        hdr.setStyleSheet(
            "color:#e8a030; font-size:14px; font-weight:bold; "
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

        # Main tab widget
        tabs = QTabWidget()
        root.addWidget(tabs)

        # ── Tab 1: Control ────────────────────────────────────────────────
        control_tab = QWidget()
        tabs.addTab(control_tab, "CONTROL")
        cols = QHBoxLayout(control_tab)
        cols.setSpacing(10)
        left  = QVBoxLayout()
        right = QVBoxLayout()
        cols.addLayout(left,  45)
        cols.addLayout(right, 55)

        # LEFT: miniPC
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
        self.minipc_btn = self._make_btn(
            "LAUNCH MINI PC  (joy + drive)", "#1a1e10", "#4a6020", "#80aa30")
        self.minipc_btn.clicked.connect(self._launch_minipc)
        ml.addWidget(self.minipc_btn)
        self.cameras_btn = self._make_btn(
            "LAUNCH CAMERAS / NAV", "#101820", "#1a4060", "#2a80c0")
        self.cameras_btn.clicked.connect(self._launch_cameras)
        ml.addWidget(self.cameras_btn)
        left.addWidget(minipc_box)

        rviz_card = ProcessCard("VISUALIZATION  ·  RViz2", self._rviz_cmd)
        left.addWidget(rviz_card)

        auto_box = QGroupBox("AUTONOMY")
        al = QVBoxLayout(auto_box)
        nav_btn  = self._make_btn("Point-Click Navigation",
                                  "#101820", "#1a4060", "#2a80c0")
        slam_btn = self._make_btn("SLAM / Mapping",
                                  "#101820", "#1a4060", "#2a80c0")
        nav_btn.clicked.connect(self._start_nav)
        slam_btn.clicked.connect(self._start_slam)
        al.addWidget(nav_btn)
        al.addWidget(slam_btn)
        left.addWidget(auto_box)

        log_box = QGroupBox("SYSTEM LOG")
        ll = QVBoxLayout(log_box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFixedHeight(90)
        ll.addWidget(self.log_view)
        left.addWidget(log_box)

        stop_all = self._make_btn(
            "⬛  STOP ALL PROCESSES", "#1a0808", "#601010", "#aa2020")
        stop_all.clicked.connect(self._stop_all)
        left.addWidget(stop_all)
        left.addStretch()

        # RIGHT: Teleop
        tbox = QGroupBox("TELEOP  ·  TANK DRIVE")
        tl = QVBoxLayout(tbox)
        tl.setSpacing(6)

        note = QLabel(
            "🎮  Left stick Y = LEFT wheels  ·  Right stick Y = RIGHT wheels")
        note.setStyleSheet("color:#3a8a50; font-size:9px; padding:2px 0;")
        tl.addWidget(note)

        tr = QHBoxLayout()
        self.teleop_led = StatusLED("off")
        tr.addWidget(self.teleop_led)
        self.teleop_btn = self._make_btn(
            "START TELEOP", "#1d3020", "#3a7a40", "#50aa60")
        self.teleop_btn.setCheckable(True)
        self.teleop_btn.clicked.connect(self._toggle_teleop)
        tr.addWidget(self.teleop_btn)
        self.estop_btn = self._make_btn(
            "E-STOP", "#300d0d", "#a02020", "#ff4040")
        self.estop_btn.clicked.connect(self._emergency_stop)
        tr.addWidget(self.estop_btn)
        tl.addLayout(tr)

        ctrl_info = QLabel(
            "LB=L spd+  LT=L spd-  ·  RB=R spd+  RT=R spd-\n"
            "A=DUMP  Y=DRIVE  B=DIG  ·  D←=CCW  D→=CW  ·  Start=estop")
        ctrl_info.setStyleSheet("color:#3a5060; font-size:9px; padding:2px 0;")
        tl.addWidget(ctrl_info)

        self.dual_speed = DualSpeedWidget()
        self.dual_speed.left_changed.connect(self._left_speed_changed)
        self.dual_speed.right_changed.connect(self._right_speed_changed)
        tl.addWidget(self.dual_speed)

        right.addWidget(tbox)

        # RIGHT: Servo
        servo_box = QGroupBox("SERVO  ·  D-pad ← CCW  /  D-pad → CW  (hold)")
        sl = QVBoxLayout(servo_box)
        sl.setSpacing(6)
        self.servo_state_label = QLabel("State: STOP")
        self.servo_state_label.setStyleSheet(
            "color:#e8a030; font-size:10px; font-weight:bold;")
        sl.addWidget(self.servo_state_label)
        self.servo_gauge = ServoGauge()
        sl.addWidget(self.servo_gauge)
        right.addWidget(servo_box)

        # RIGHT: Actuators
        abox = QGroupBox("ACTUATORS  ·  encoder-based positions")
        al2 = QVBoxLayout(abox)
        ah = QLabel("A=DUMP  ·  Y=DRIVE  ·  B=DIG")
        ah.setStyleSheet("color:#405060; font-size:9px;")
        al2.addWidget(ah)
        abr = QHBoxLayout()
        self.act_dump_btn  = self._make_btn("DUMP  (A)",  "#1a2820", "#2a6040", "#40c070")
        self.act_drive_btn = self._make_btn("DRIVE (Y)",  "#1a2028", "#2a4060", "#4070c0")
        self.act_dig_btn   = self._make_btn("DIG   (B)",  "#281a1a", "#602a2a", "#c04040")
        abr.addWidget(self.act_dump_btn)
        abr.addWidget(self.act_drive_btn)
        abr.addWidget(self.act_dig_btn)
        al2.addLayout(abr)
        self.act_status = QLabel("Actuator: idle")
        self.act_status.setStyleSheet("color:#607080; font-size:9px;")
        al2.addWidget(self.act_status)
        right.addWidget(abox)
        right.addStretch()

        # ── Tab 2: Debug ──────────────────────────────────────────────────
        debug_tab = QWidget()
        tabs.addTab(debug_tab, "DEBUG")
        debug_lay = QVBoxLayout(debug_tab)
        self.debug_panel = DebugPanel()
        debug_lay.addWidget(self.debug_panel)

        self._log("Mission Control ready  ·  TANK DRIVE mode")
        self._log("LB/LT = left spd  ·  RB/RT = right spd  ·  A/Y/B = actuator pos")

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
        """Thread-safe: can be called from any thread."""
        self.log_signal.emit(str(msg))

    def _log_direct(self, msg):
        """Must only be called on the main thread (via log_signal)."""
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.append(
            f"<span style='color:#304050'>[{ts}]</span> {msg}")

    # ── Status subscriber ─────────────────────────────────────────────────

    def _start_status_subscriber(self):
        self._status_thread = JoyStatusSubscriber()
        self._status_thread.status_received.connect(
            self.debug_panel.update_status)
        self._status_thread.start()

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
            # Use signals — we're in a background thread
            self.log_signal.emit(f"miniPC {MINIPC_IP}: {'online' if ok else 'offline'}")
        threading.Thread(target=run, daemon=True).start()

    # ── MiniPC launch ─────────────────────────────────────────────────────

    def _launch_minipc(self):
        """
        SSH into miniPC and run exactly:
            ros2 run joy joy_node &
            python3 ~/lunar_rover_ws/joy_to_arduino.py
        Waits for joy_node to be ready before starting joy_to_arduino.
        Output is streamed live to the GUI log.
        """
        self._log("SSH → miniPC: launching joy_node + joy_to_arduino…")
        self.minipc_led.set_color("yellow")
        self.minipc_status.setText("Starting…")

        # Use tmux so both processes run in persistent sessions with full stdin/stdout.
        # This fixes the "must be listening" issue — tmux keeps the pty alive.
        remote_script = (
            "source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null\n"
            f"[ -f {MINIPC_WS}/install/setup.bash ] && source {MINIPC_WS}/install/setup.bash\n"
            "export ROS_DOMAIN_ID=42\n"
            "export ROS_LOCALHOST_ONLY=0\n"
            "export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET\n"
            "\n"
            "# Kill stale tmux sessions and processes\n"
            "echo '[miniPC] killing stale processes...'\n"
            "tmux kill-session -t joy_node 2>/dev/null\n"
            "tmux kill-session -t joy_arduino 2>/dev/null\n"
            "pkill -f joy_to_arduino 2>/dev/null\n"
            "pkill -f joy_node 2>/dev/null\n"
            "sleep 1\n"
            "\n"
            "# Build env export string for tmux sessions\n"
            "ROS_SETUP='source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null'\n"
            f"WS_SETUP='[ -f {MINIPC_WS}/install/setup.bash ] && source {MINIPC_WS}/install/setup.bash'\n"
            "ENV_EXPORTS='export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=0; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET'\n"
            "\n"
            "# Launch joy_node in tmux session 'joy_node'\n"
            "echo '[miniPC] starting joy_node in tmux...'\n"
            "tmux new-session -d -s joy_node -x 220 -y 50\n"
            "tmux send-keys -t joy_node \"$ROS_SETUP; $WS_SETUP; $ENV_EXPORTS; ros2 run joy joy_node\" Enter\n"
            "\n"
            "# Wait for /joy topic\n"
            "JOY_UP=false\n"
            "for i in 1 2 3 4 5 6 7 8; do\n"
            "  sleep 1\n"
            "  ros2 topic list 2>/dev/null | grep -q '^/joy$' && JOY_UP=true && echo \"[miniPC] /joy topic live (${i}s)\" && break\n"
            "  echo \"[miniPC] waiting for joy_node... (${i}s)\"\n"
            "done\n"
            "if [ \"$JOY_UP\" = \"false\" ]; then\n"
            "  echo '[miniPC] ✗ joy_node failed — is controller plugged in? (ls /dev/input/js*)'\n"
            "  exit 1\n"
            "fi\n"
            "\n"
            "# Launch joy_to_arduino in tmux session 'joy_arduino'\n"
            "echo '[miniPC] starting joy_to_arduino in tmux...'\n"
            "tmux new-session -d -s joy_arduino -x 220 -y 50\n"
            f"tmux send-keys -t joy_arduino \"$ROS_SETUP; $WS_SETUP; $ENV_EXPORTS; python3 {MINIPC_WS}/joy_to_arduino.py 2>&1 | tee /tmp/rover_arduino.log\" Enter\n"
            "\n"
            "sleep 3\n"
            "# Check if session is still running\n"
            "tmux list-sessions 2>/dev/null | grep -q joy_arduino \\\n"
            "  && echo '[miniPC] ✓ joy_to_arduino running' \\\n"
            "  || echo '[miniPC] ✗ joy_to_arduino session died'\n"
            "echo '[miniPC] --- arduino log ---'\n"
            "tail -10 /tmp/rover_arduino.log 2>/dev/null || echo '(log not yet written)'\n"
            "echo '[miniPC] To watch live: ssh cheese@192.168.0.102 then: tmux attach -t joy_arduino'\n"
        )

        ssh_cmd = (
            f'ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=no '
            f'{MINIPC_USER}@{MINIPC_IP} bash -s'
        )

        def run():
            try:
                proc = subprocess.Popen(
                    ssh_cmd.split(),
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True
                )
                stdout, _ = proc.communicate(input=remote_script)
                success = False
                for line in stdout.splitlines():
                    self._log(line)
                    if '✓ joy_to_arduino running' in line:
                        success = True
                if success:
                    self._log("✓ miniPC teleop ready — use controller to drive")
                    QTimer.singleShot(0, lambda: self.minipc_led.set_color("green"))
                    QTimer.singleShot(0, lambda: self.minipc_status.setText("joy_node + joy_to_arduino running"))
                else:
                    self._log("⚠ Check /tmp/rover_arduino.log and /tmp/rover_joy_node.log on miniPC")
                    QTimer.singleShot(0, lambda: self.minipc_led.set_color("red"))
                    QTimer.singleShot(0, lambda: self.minipc_status.setText("launch failed — check log"))
            except Exception as e:
                self._log(f"SSH error: {e}")

        threading.Thread(target=run, daemon=True).start()

    def _launch_cameras(self):
        """Run full_launch_minipc.sh for cameras and autonomous nav."""
        try:
            delay = float(self.delay_input.text() or "0")
        except ValueError:
            delay = 0.0
        delay_str = f"{delay:.1f}"
        mode = f"competition delay {delay_str}s" if delay > 0 else "live mode"
        self._log(f"SSH → miniPC: launching cameras + nav… ({mode})")
        cmd = (
            f'ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=no '
            f'{MINIPC_USER}@{MINIPC_IP} '
            f'"DELAY_SEC={delay_str} nohup bash {MINIPC_WS}/full_launch_minipc.sh '
            f'> /tmp/minipc_launch.log 2>&1 &"'
        )
        def run():
            r = subprocess.run(cmd, shell=True)
            if r.returncode == 0:
                self._log("Cameras/nav launch sent ✓  (tail /tmp/minipc_launch.log on miniPC)")
            else:
                self._log("SSH failed — is miniPC reachable?")
        threading.Thread(target=run, daemon=True).start()

    # ── RViz ─────────────────────────────────────────────────────────────

    def _rviz_cmd(self):
        return f"rviz2 -d {RVIZ_CONFIG}" if os.path.exists(RVIZ_CONFIG) \
               else "rviz2"

    # ── Autonomy ─────────────────────────────────────────────────────────

    def _start_nav(self):
        self._log("Launching point-click navigation…")
        subprocess.Popen(
            "bash -c 'source /opt/ros/$(ls /opt/ros)/setup.bash && "
            "source ~/lunar_rover_ws/install/setup.bash 2>/dev/null; "
            "ros2 launch lunar_robot_hardware arduino_navigation.launch.py'",
            shell=True)

    def _start_slam(self):
        self._log("Launching SLAM…")
        subprocess.Popen(
            f'ssh {MINIPC_USER}@{MINIPC_IP} "bash {MINIPC_WS}/slam_minipc.sh map"',
            shell=True)

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
        # Wire debug signals
        self._teleop_thread.joy_raw_signal.connect(
            self.debug_panel.update_joy_raw)
        self._teleop_thread.arduino_tx_signal.connect(
            self.debug_panel.log_tx)
        self._teleop_thread.set_speed_left(
            self.dual_speed.left_slider.value() / 100.0)
        self._teleop_thread.set_speed_right(
            self.dual_speed.right_slider.value() / 100.0)
        self._teleop_thread.start()
        self._teleop_active = True
        self.teleop_led.set_color("green")
        self.teleop_btn.setText("STOP TELEOP")
        self._log("Teleop started  ·  GUI is DISPLAY ONLY")
        self._log("All commands (A/Y/B/bumpers/triggers/dpad) handled by joy_to_arduino.py on miniPC")
        self._log("Check DEBUG tab for live /joy data and command echo")

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
        self.debug_panel.log_tx('[GUI] E-STOP button pressed')

    # ── Speed callbacks ───────────────────────────────────────────────────

    def _on_ctrl_speed_left(self, spd: float):
        self.dual_speed.set_left(spd)

    def _on_ctrl_speed_right(self, spd: float):
        self.dual_speed.set_right(spd)

    def _left_speed_changed(self, val: float):
        if self._teleop_thread:
            self._teleop_thread.set_speed_left(val)

    def _right_speed_changed(self, val: float):
        if self._teleop_thread:
            self._teleop_thread.set_speed_right(val)

    # ── Servo callbacks ───────────────────────────────────────────────────

    def _on_ctrl_servo(self, angle: int):
        self.servo_gauge.set_angle(angle)
        if angle == SERVO_CW:
            self.servo_state_label.setText("State: CW →")
            self.servo_state_label.setStyleSheet(
                "color:#e8a030; font-size:10px; font-weight:bold;")
        elif angle == SERVO_CCW:
            self.servo_state_label.setText("State: ← CCW")
            self.servo_state_label.setStyleSheet(
                "color:#50c8ff; font-size:10px; font-weight:bold;")
        else:
            self.servo_state_label.setText("State: STOP")
            self.servo_state_label.setStyleSheet(
                "color:#607080; font-size:10px; font-weight:bold;")

    # ── Stop all ─────────────────────────────────────────────────────────

    def _stop_all(self):
        self._stop_teleop()
        self._log("Stopping all processes…")
        subprocess.run(
            "pkill -f rviz2; pkill -f unified_navigator; pkill -f slam_minipc",
            shell=True)

    # ── Cleanup ───────────────────────────────────────────────────────────

    def closeEvent(self, event):
        if self._status_thread:
            self._status_thread.stop()
        super().closeEvent(event)


# ═════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("Courier New", 10))
    w = MissionControl()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()