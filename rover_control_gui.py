#!/usr/bin/env python3
"""
Lunar Rover Mission Control GUI  —  rover_control_gui.py
Fixes: resizable layout, updated button labels for d-pad actuator control.
"""

import os, sys, math, subprocess, threading, time

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit, QSlider,
    QSizePolicy, QFrame, QLineEdit, QTabWidget, QSpacerItem
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

# ── Controller mapping (must match joy_to_arduino.py) ─────────────────────
AXIS_LEFT         = 1
AXIS_RIGHT        = 4
AXIS_LT           = 2
AXIS_RT           = 5
TRIGGER_THRESHOLD = 0.5   # axis < this → pressed

BTN_LB    = 4
BTN_RB    = 5
BTN_A     = 0   # DUMP
BTN_Y     = 3   # DRIVE
BTN_B     = 1   # DIG
BTN_X     = 2   # CALIBRATE
BTN_START = 7

DPAD_AXIS_LR = 6   # servo
DPAD_AXIS_UD = 7   # actuator manual


# ═════════════════════════════════════════════════════════════════════════
# SERVO GAUGE WIDGET
# ═════════════════════════════════════════════════════════════════════════

class ServoGauge(QWidget):
    """Arc gauge — shows CW/CCW/STOP state (360° servo, no angle tracking)."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._state = 'STOP'   # 'CW', 'CCW', 'STOP'
        self._anim  = 0.0      # animated sweep 0–1
        self._anim_timer = QTimer()
        self._anim_timer.timeout.connect(self._step_anim)
        self.setMinimumSize(140, 80)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

    def set_state(self, state: str):
        self._state = state
        if state != 'STOP' and not self._anim_timer.isActive():
            self._anim_timer.start(50)
        elif state == 'STOP':
            self._anim_timer.stop()
            self._anim = 0.0
            self.update()

    def _step_anim(self):
        self._anim = (self._anim + 0.04) % 1.0
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx = w // 2
        cy = h - 8
        r  = min(cx - 4, h - 12)

        # Background arc
        arc_pen = QPen(QColor(30, 38, 52), 8)
        arc_pen.setCapStyle(Qt.RoundCap)
        p.setPen(arc_pen)
        p.drawArc(QRect(cx-r, cy-r, r*2, r*2), 0, 180*16)

        # Coloured arc segment that sweeps to show direction
        if self._state != 'STOP':
            sweep = int(self._anim * 180) if self._state == 'CW' else \
                    int((1.0 - self._anim) * 180)
            col = QColor(232, 160, 48) if self._state == 'CW' else QColor(80, 200, 255)
            fp = QPen(col, 8); fp.setCapStyle(Qt.RoundCap)
            p.setPen(fp)
            p.drawArc(QRect(cx-r, cy-r, r*2, r*2), 180*16, sweep*16)

        # Label
        lut = {'CW': ('CW →', '#e8a030'), 'CCW': ('← CCW', '#50c8ff'),
               'STOP': ('STOP', '#506070')}
        text, col = lut.get(self._state, ('?', '#ffffff'))
        p.setPen(QPen(QColor(col)))
        p.setFont(QFont('Courier New', 9, QFont.Bold))
        p.drawText(QRect(0, 0, w, h - 2),
                   Qt.AlignHCenter | Qt.AlignBottom, text)


# ═════════════════════════════════════════════════════════════════════════
# TELEOP PUBLISHER
# ═════════════════════════════════════════════════════════════════════════

class TeleopPublisher(QThread):
    status_changed      = pyqtSignal(str)
    speed_left_changed  = pyqtSignal(float)
    speed_right_changed = pyqtSignal(float)
    servo_state_signal  = pyqtSignal(str)    # 'CW', 'CCW', 'STOP'
    joy_raw_signal      = pyqtSignal(str)
    arduino_tx_signal   = pyqtSignal(str)

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
        self._want_act = 0

        self._sent_lin = None
        self._sent_ang = None
        self._sent_act = None

        self._prev_btns    = {}
        self._prev_dpad_lr = 0.0
        self._prev_dpad_ud = 0.0
        self._lt_prev      = 1.0
        self._rt_prev      = 1.0

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

    def _joy_cb(self, msg):
        """GUI is DISPLAY ONLY — no commands sent from here."""
        try:
            ax  = lambda i: msg.axes[i]    if i < len(msg.axes)    else 0.0
            btn = lambda i: msg.buttons[i] if i < len(msg.buttons) else 0

            axes_str = ' '.join(f'{ax(i):+.2f}' for i in range(len(msg.axes)))
            btns_str = ' '.join(str(btn(i)) for i in range(len(msg.buttons)))
            self.joy_raw_signal.emit(f'axes=[{axes_str}]  btns=[{btns_str}]')

            # Button echo for debug panel
            if self._rising(BTN_LB, btn(BTN_LB)):
                self.arduino_tx_signal.emit('[sees] LB → left spd+ (miniPC)')
            if self._rising(BTN_RB, btn(BTN_RB)):
                self.arduino_tx_signal.emit('[sees] RB → right spd+ (miniPC)')

            # Trigger edge for speed down display
            lt_cur = ax(AXIS_LT)
            if lt_cur < TRIGGER_THRESHOLD and self._lt_prev >= TRIGGER_THRESHOLD:
                self.arduino_tx_signal.emit('[sees] LT pressed → left spd- (miniPC)')
            self._lt_prev = lt_cur

            rt_cur = ax(AXIS_RT)
            if rt_cur < TRIGGER_THRESHOLD and self._rt_prev >= TRIGGER_THRESHOLD:
                self.arduino_tx_signal.emit('[sees] RT pressed → right spd- (miniPC)')
            self._rt_prev = rt_cur

            if self._rising(BTN_A, btn(BTN_A)):
                self.arduino_tx_signal.emit('[sees] A → DUMP cmd (miniPC)')
                self.status_changed.emit('A → DUMP (miniPC)')
            if self._rising(BTN_Y, btn(BTN_Y)):
                self.arduino_tx_signal.emit('[sees] Y → DRIVE cmd (miniPC)')
                self.status_changed.emit('Y → DRIVE (miniPC)')
            if self._rising(BTN_B, btn(BTN_B)):
                self.arduino_tx_signal.emit('[sees] B → DIG cmd (miniPC)')
                self.status_changed.emit('B → DIG (miniPC)')
            if self._rising(BTN_X, btn(BTN_X)):
                self.arduino_tx_signal.emit('[sees] X → CALIBRATE cmd (miniPC)')
                self.status_changed.emit('X → CALIBRATE (miniPC)')

            # D-pad LR → servo
            cur_lr = ax(DPAD_AXIS_LR)
            if cur_lr > 0.5 and self._prev_dpad_lr <= 0.5:
                self.servo_state_signal.emit('CW')
                self.arduino_tx_signal.emit('[sees] D→ → Servo CW (miniPC)')
            elif cur_lr < -0.5 and self._prev_dpad_lr >= -0.5:
                self.servo_state_signal.emit('CCW')
                self.arduino_tx_signal.emit('[sees] D← → Servo CCW (miniPC)')
            elif abs(cur_lr) < 0.5 and abs(self._prev_dpad_lr) > 0.5:
                self.servo_state_signal.emit('STOP')
                self.arduino_tx_signal.emit('[sees] D-pad LR release → Servo STOP (miniPC)')
            self._prev_dpad_lr = cur_lr

            # D-pad UD → actuator manual
            cur_ud = ax(DPAD_AXIS_UD)
            if cur_ud > 0.5 and self._prev_dpad_ud <= 0.5:
                self.arduino_tx_signal.emit('[sees] D↑ → Act EXTEND held (miniPC)')
                self.status_changed.emit('D↑ → Actuator EXTEND')
            elif cur_ud < -0.5 and self._prev_dpad_ud >= -0.5:
                self.arduino_tx_signal.emit('[sees] D↓ → Act RETRACT held (miniPC)')
                self.status_changed.emit('D↓ → Actuator RETRACT')
            elif abs(cur_ud) < 0.5 and abs(self._prev_dpad_ud) > 0.5:
                self.arduino_tx_signal.emit('[sees] D-pad UD release → Act STOP (miniPC)')
            self._prev_dpad_ud = cur_ud

        except Exception as e:
            self.status_changed.emit(f'Joy display error: {e}')

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

    def run(self):
        if not ROS_AVAILABLE:
            self.status_changed.emit('ROS2 not available')
            return
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node('rover_laptop_teleop')
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
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
    COLORS = {'off':    QColor(40, 42, 50),
              'green':  QColor(60, 220, 80),
              'yellow': QColor(255, 200, 40),
              'red':    QColor(220, 60, 60)}

    def __init__(self, color='off', parent=None):
        super().__init__(parent)
        self.setFixedSize(14, 14)
        self.set_color(color)

    def set_color(self, color):
        self._color = color
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = self.COLORS.get(self._color, self.COLORS['off'])
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
        self.led = StatusLED('off')
        row.addWidget(self.led)
        self.btn = QPushButton('START')
        self.btn.setCheckable(True)
        self.btn.setMinimumHeight(28)
        self.btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn.clicked.connect(self._toggle)
        row.addWidget(self.btn)
        lay.addLayout(row)
        self.log = QLabel('—')
        self.log.setStyleSheet('color:#506070; font-size:8px;')
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
        self.led.set_color('green')
        self.btn.setText('STOP')
        self.log.setText(f'PID {self._proc.pid}')

    def _stop(self):
        if self._proc:
            self._proc.terminate()
            self._proc = None
        self.led.set_color('off')
        self.btn.setText('START')
        self.btn.setChecked(False)
        self.log.setText('stopped')


# ═════════════════════════════════════════════════════════════════════════
# DUAL SPEED SLIDER WIDGET
# ═════════════════════════════════════════════════════════════════════════

class DualSpeedWidget(QGroupBox):
    left_changed  = pyqtSignal(float)
    right_changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__('SPEED  ·  independent L / R', parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self._build()

    def _build(self):
        root = QHBoxLayout()
        root.setSpacing(12)

        def make_side(label_text, color):
            col = QVBoxLayout()
            col.setSpacing(3)
            hdr = QLabel(label_text)
            hdr.setAlignment(Qt.AlignCenter)
            hdr.setStyleSheet(
                f'color:{color}; font-size:9px; font-weight:bold; letter-spacing:1px;')
            col.addWidget(hdr)
            slider = QSlider(Qt.Vertical)
            slider.setRange(5, 100)
            slider.setValue(50)
            slider.setMinimumHeight(80)
            slider.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
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
            val_lbl = QLabel('0.50')
            val_lbl.setAlignment(Qt.AlignCenter)
            val_lbl.setStyleSheet(
                f'color:{color}; font-size:11px; font-weight:bold;')
            slider_row = QHBoxLayout()
            slider_row.addStretch()
            slider_row.addWidget(slider)
            slider_row.addStretch()
            col.addLayout(slider_row, 1)
            col.addWidget(val_lbl)
            return col, slider, val_lbl

        left_col,  self.left_slider,  self.left_val  = make_side(
            '◀  LEFT  (LB/LT)', '#50c878')
        right_col, self.right_slider, self.right_val = make_side(
            'RIGHT  ▶ (RB/RT)', '#e8a030')

        div = QFrame()
        div.setFrameShape(QFrame.VLine)
        div.setStyleSheet('color:#1a2030;')

        root.addLayout(left_col,  1)
        root.addWidget(div)
        root.addLayout(right_col, 1)

        self.left_slider.valueChanged.connect(self._on_left)
        self.right_slider.valueChanged.connect(self._on_right)

        sync_row = QHBoxLayout()
        self._sync_btn = QPushButton('⟺  Sync both')
        self._sync_btn.setMinimumHeight(22)
        self._sync_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._sync_btn.setStyleSheet("""
            QPushButton {
                background:#0e1018; color:#506070;
                border:1px solid #1a2030; border-radius:3px;
                font-size:8px; padding:2px 6px;
            }
            QPushButton:hover { color:#a0b8c8; border-color:#2a3040; }
        """)
        self._sync_btn.clicked.connect(self._sync_to_left)
        sync_row.addWidget(self._sync_btn)

        outer = QVBoxLayout(self)
        outer.setSpacing(4)
        inner_widget = QWidget()
        inner_widget.setLayout(root)
        outer.addWidget(inner_widget, 1)
        outer.addLayout(sync_row)

    def _on_left(self, val):
        v = val / 100.0
        self.left_val.setText(f'{v:.2f}')
        self.left_changed.emit(v)

    def _on_right(self, val):
        v = val / 100.0
        self.right_val.setText(f'{v:.2f}')
        self.right_changed.emit(v)

    def _sync_to_left(self):
        self.right_slider.setValue(self.left_slider.value())

    def set_left(self, v: float):
        self.left_slider.blockSignals(True)
        self.left_slider.setValue(int(v * 100))
        self.left_val.setText(f'{v:.2f}')
        self.left_slider.blockSignals(False)

    def set_right(self, v: float):
        self.right_slider.blockSignals(True)
        self.right_slider.setValue(int(v * 100))
        self.right_val.setText(f'{v:.2f}')
        self.right_slider.blockSignals(False)


# ═════════════════════════════════════════════════════════════════════════
# DEBUG PANEL
# ═════════════════════════════════════════════════════════════════════════

class DebugPanel(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('DEBUG  ·  live telemetry', parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._build()
        self._joy_count    = 0
        self._last_joy_time = time.monotonic()

    def _build(self):
        lay = QVBoxLayout(self)
        lay.setSpacing(4)

        self._status_label = QLabel('Waiting for status…')
        self._status_label.setStyleSheet(
            'color:#50c8a0; font-size:9px; font-family:monospace;')
        self._status_label.setWordWrap(True)
        lay.addWidget(self._status_label)

        hint = QLabel(
            '  A=DUMP  Y=DRIVE  B=DIG  |  LB=L spd+  LT=L spd-  '
            'RB=R spd+  RT=R spd-  |  D←→=Servo  D↑↓=Act manual  |  Start=estop')
        hint.setStyleSheet('color:#3a6060; font-size:8px;')
        hint.setWordWrap(True)
        lay.addWidget(hint)

        joy_hdr = QLabel('Controller raw (every ~1.5s):')
        joy_hdr.setStyleSheet('color:#506070; font-size:8px;')
        lay.addWidget(joy_hdr)

        self._joy_raw = QLabel('—  (no /joy messages yet)')
        self._joy_raw.setStyleSheet(
            'color:#4080c0; font-size:8px; font-family:monospace; '
            'background:#09111a; padding:2px 4px; border-radius:3px;')
        self._joy_raw.setWordWrap(True)
        lay.addWidget(self._joy_raw)

        tx_hdr = QLabel('Command events (last 12):')
        tx_hdr.setStyleSheet('color:#506070; font-size:8px;')
        lay.addWidget(tx_hdr)

        self._tx_log = QTextEdit()
        self._tx_log.setReadOnly(True)
        self._tx_log.setMinimumHeight(80)
        self._tx_log.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._tx_log.setStyleSheet(
            'background:#060e08; color:#40c060; '
            'border:1px solid #1a3020; border-radius:3px; '
            'font-family:monospace; font-size:8px;')
        lay.addWidget(self._tx_log, 1)

        health_row = QHBoxLayout()
        self._joy_led    = StatusLED('off')
        self._joy_health = QLabel('No /joy msgs')
        self._joy_health.setStyleSheet('color:#506070; font-size:9px;')
        health_row.addWidget(self._joy_led)
        health_row.addWidget(self._joy_health)
        health_row.addStretch()
        lay.addLayout(health_row)

        self._health_timer = QTimer()
        self._health_timer.timeout.connect(self._check_joy_health)
        self._health_timer.start(2000)

    def update_status(self, status_str: str):
        parts = {}
        for item in status_str.split('|'):
            if '=' in item:
                k, v = item.split('=', 1)
                parts[k.strip()] = v.strip()
        serial_ok = parts.get('serial', '?') == 'True'
        color = '#50c8a0' if serial_ok else '#c85050'
        self._status_label.setStyleSheet(
            f'color:{color}; font-size:9px; font-family:monospace;')
        self._status_label.setText(
            f"serial={'OK' if serial_ok else 'NO PORT'}  "
            f"spd L={parts.get('spd_L','?')} R={parts.get('spd_R','?')}  "
            f"enc={parts.get('enc','?')}  estop={parts.get('estop','?')}")

    def update_joy_raw(self, raw_str: str):
        self._joy_raw.setText(raw_str)
        self._joy_count += 1
        self._last_joy_time = time.monotonic()
        self._joy_led.set_color('green')
        self._joy_health.setText(f'Joy OK  ({self._joy_count} msgs)')
        self._joy_health.setStyleSheet('color:#50c870; font-size:9px;')

    def log_tx(self, msg: str):
        from datetime import datetime
        ts  = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        cur = self._tx_log.toPlainText().splitlines()
        lines = (cur + [f'[{ts}] {msg}'])[-12:]
        self._tx_log.setPlainText('\n'.join(lines))
        sb = self._tx_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _check_joy_health(self):
        age = time.monotonic() - self._last_joy_time
        if age > 3.0:
            self._joy_led.set_color('red')
            self._joy_health.setText(f'No /joy for {age:.0f}s ⚠')
            self._joy_health.setStyleSheet('color:#c05050; font-size:9px;')


# ═════════════════════════════════════════════════════════════════════════
# DELAY STATUS SUBSCRIBER THREAD
# ═════════════════════════════════════════════════════════════════════════

class DelayStatusThread(QThread):
    """Subscribes to /delay_status (JSON string) published by joy_to_arduino.py."""
    status_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self._running = False

    def run(self):
        if not ROS_AVAILABLE:
            return
        try:
            import json
            if not rclpy.ok():
                rclpy.init()
            node = rclpy.create_node('delay_gui_sub')
            node.create_subscription(
                String, '/delay_status',
                lambda msg: self._emit(msg.data), 10)
            self._running = True
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f'[DelayStatusThread] {e}', flush=True)

    def _emit(self, data_str: str):
        try:
            import json
            self.status_received.emit(json.loads(data_str))
        except Exception:
            pass

    def stop(self):
        self._running = False
        self.quit()
        self.wait(2000)


# ═════════════════════════════════════════════════════════════════════════
# DELAY CONTROL WIDGET
# ═════════════════════════════════════════════════════════════════════════

class DelayControlWidget(QGroupBox):
    """
    Shows a toggle to enable/disable the 5-second lunar comms delay,
    and a command timer that goes RED (< 5 s) then GREEN (>= 5 s).
    Receives live data from /delay_status topic via DelayStatusThread.
    """

    DELAY_SEC = 5.0

    def __init__(self, toggle_cb, parent=None):
        super().__init__('COMMS DELAY  ·  lunar simulation  (5 s each way)', parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._toggle_cb    = toggle_cb
        self._enabled      = False
        self._last_cmd_age = None   # seconds, from /delay_status
        self._pending      = 0
        self._build_ui()

        # 10 Hz poll to update the elapsed timer display
        self._poll = QTimer()
        self._poll.timeout.connect(self._tick)
        self._poll.start(100)

    # ── UI ────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(5)
        root.setContentsMargins(8, 12, 8, 8)

        # Toggle row
        toggle_row = QHBoxLayout()
        self._toggle_btn = QPushButton('ENABLE  5 s  DELAY')
        self._toggle_btn.setCheckable(True)
        self._toggle_btn.setMinimumHeight(28)
        self._toggle_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._toggle_btn.clicked.connect(self._on_toggle)
        self._toggle_btn.setStyleSheet(self._btn_style(active=False))
        toggle_row.addWidget(self._toggle_btn)

        self._state_badge = QLabel('OFF')
        self._state_badge.setAlignment(Qt.AlignCenter)
        self._state_badge.setFixedWidth(42)
        self._state_badge.setStyleSheet(
            'color:#506070; font-size:9px; font-weight:bold; '
            'background:#0e1018; border:1px solid #1a2030; border-radius:4px; '
            'padding:2px;')
        toggle_row.addWidget(self._state_badge)
        root.addLayout(toggle_row)

        # Separator
        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet('color:#1a2030;')
        root.addWidget(sep)

        # Timer label
        timer_hdr = QLabel('Time since last command sent')
        timer_hdr.setStyleSheet('color:#405060; font-size:8px;')
        root.addWidget(timer_hdr)

        self._timer_display = QLabel('—')
        self._timer_display.setAlignment(Qt.AlignCenter)
        self._timer_display.setFont(QFont('Courier New', 22, QFont.Bold))
        self._timer_display.setMinimumHeight(50)
        self._timer_display.setStyleSheet(
            'color:#506070; font-size:22px; font-weight:bold; '
            'background:#090b10; border:1px solid #1a2030; '
            'border-radius:6px; padding:4px 0;')
        root.addWidget(self._timer_display)

        self._timer_hint = QLabel('')
        self._timer_hint.setAlignment(Qt.AlignCenter)
        self._timer_hint.setStyleSheet('color:#304050; font-size:9px;')
        root.addWidget(self._timer_hint)

        # Pending counter row
        pend_row = QHBoxLayout()
        pend_lbl = QLabel('Commands in transit:')
        pend_lbl.setStyleSheet('color:#405060; font-size:8px;')
        pend_row.addWidget(pend_lbl)
        self._pending_lbl = QLabel('0')
        self._pending_lbl.setStyleSheet(
            'color:#50c0f0; font-size:8px; font-weight:bold;')
        pend_row.addWidget(self._pending_lbl)
        pend_row.addStretch()
        root.addLayout(pend_row)

        note = QLabel(
            'Delay applies to drive, actuator & servo commands.\n'
            'Camera feed also delayed 5 s when enabled.')
        note.setStyleSheet('color:#2a3c50; font-size:8px;')
        root.addWidget(note)

    # ── Slot: receives parsed JSON from DelayStatusThread ─────────────

    def on_status_update(self, payload: dict):
        self._last_cmd_age = payload.get('last_cmd_age')
        self._pending      = payload.get('pending', 0)
        remote_enabled     = payload.get('delay_enabled', False)
        if remote_enabled != self._enabled:
            self._enabled = remote_enabled
            self._toggle_btn.setChecked(remote_enabled)
            self._apply_toggle_style(remote_enabled)

    # ── 10 Hz tick ────────────────────────────────────────────────────

    def _tick(self):
        self._pending_lbl.setText(str(self._pending))

        if not self._enabled or self._last_cmd_age is None:
            self._timer_display.setText('—')
            self._timer_display.setStyleSheet(
                'color:#506070; font-size:22px; font-weight:bold; '
                'background:#090b10; border:1px solid #1a2030; '
                'border-radius:6px; padding:4px 0;')
            self._timer_hint.setText(
                'Enable delay to start timing' if not self._enabled
                else 'Waiting for first command…')
            return

        age       = self._last_cmd_age
        delivered = age >= self.DELAY_SEC

        display_text = f'{age:.1f} s' if age < 100 else f'{int(age)} s'

        if delivered:
            color  = '#28d060'
            bg     = '#041208'
            border = '#0c4020'
            hint   = '✓  Rover received last command'
        else:
            remaining = self.DELAY_SEC - age
            color  = '#e84040'
            bg     = '#120808'
            border = '#401010'
            hint   = f'⏳  Rover receives command in  {remaining:.1f} s'

        self._timer_display.setText(display_text)
        self._timer_display.setStyleSheet(
            f'color:{color}; font-size:22px; font-weight:bold; '
            f'background:{bg}; border:1px solid {border}; '
            f'border-radius:6px; padding:4px 0;')
        self._timer_hint.setText(hint)
        self._timer_hint.setStyleSheet(f'color:{color}; font-size:9px;')

    # ── Toggle ────────────────────────────────────────────────────────

    def _on_toggle(self, checked: bool):
        self._enabled = checked
        self._apply_toggle_style(checked)
        if not checked:
            self._last_cmd_age = None
            self._timer_hint.setText('')
        try:
            self._toggle_cb(checked)
        except Exception as e:
            print(f'[DelayWidget] toggle error: {e}', flush=True)

    def _apply_toggle_style(self, active: bool):
        self._toggle_btn.setChecked(active)
        self._toggle_btn.setStyleSheet(self._btn_style(active))
        if active:
            self._toggle_btn.setText('DISABLE  DELAY')
            self._state_badge.setText('5 s')
            self._state_badge.setStyleSheet(
                'color:#e84040; font-size:9px; font-weight:bold; '
                'background:#120808; border:1px solid #401010; '
                'border-radius:4px; padding:2px;')
        else:
            self._toggle_btn.setText('ENABLE  5 s  DELAY')
            self._state_badge.setText('OFF')
            self._state_badge.setStyleSheet(
                'color:#506070; font-size:9px; font-weight:bold; '
                'background:#0e1018; border:1px solid #1a2030; '
                'border-radius:4px; padding:2px;')

    @staticmethod
    def _btn_style(active: bool) -> str:
        if active:
            return (
                'QPushButton {background:#1a0808; color:#e84040; '
                'border:1px solid #601010; border-radius:5px; '
                'padding:4px 10px; font-size:10px; font-weight:bold;} '
                'QPushButton:hover {background:#280c0c;} '
                'QPushButton:checked {background:#280c0c; color:#ff5050;}'
            )
        return (
            'QPushButton {background:#101820; color:#50a0c0; '
            'border:1px solid #1c3040; border-radius:5px; '
            'padding:4px 10px; font-size:10px; font-weight:bold;} '
            'QPushButton:hover {background:#162030;} '
            'QPushButton:checked {background:#162030; color:#70c0e0;}'
        )


# ═════════════════════════════════════════════════════════════════════════
# JOY STATUS SUBSCRIBER
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
                lambda msg: self.status_received.emit(msg.data), 10)
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
    log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.log_signal.connect(self._log_direct)
        self.setWindowTitle('Lunar Rover Mission Control')
        # Use a sensible minimum — window can grow freely
        self.setMinimumSize(900, 700)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self._teleop_active    = False
        self._teleop_thread    = None
        self._status_thread    = None
        self._delay_status_thread = None
        self._delay_enabled    = False
        self._delay_pub_node   = None
        self._delay_ros_pub    = None

        self._apply_stylesheet()
        self._build_ui()
        self._start_connection_checker()
        self._start_status_subscriber()

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

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(6)
        root.setContentsMargins(10, 10, 10, 10)

        # Header
        hdr = QLabel('⬡  LUNAR ROVER  ·  MISSION CONTROL')
        hdr.setStyleSheet(
            'color:#e8a030; font-size:14px; font-weight:bold; '
            'letter-spacing:3px; padding:4px 0;')
        hdr.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        root.addWidget(hdr)

        # Connection bar
        cbar = QHBoxLayout()
        self.conn_led   = StatusLED('off')
        self.conn_label = QLabel('miniPC: checking…')
        self.conn_label.setStyleSheet('color:#506070; font-size:9px;')
        cbar.addWidget(self.conn_led)
        cbar.addWidget(self.conn_label)
        cbar.addStretch()
        root.addLayout(cbar)

        # ── Tab widget (fills remaining space) ───────────────────────────
        tabs = QTabWidget()
        tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(tabs, 1)

        # ── Tab 1: Control ────────────────────────────────────────────────
        control_tab = QWidget()
        control_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tabs.addTab(control_tab, 'CONTROL')

        cols = QHBoxLayout(control_tab)
        cols.setSpacing(8)
        cols.setContentsMargins(6, 6, 6, 6)

        left  = QVBoxLayout()
        left.setSpacing(6)
        right = QVBoxLayout()
        right.setSpacing(6)

        # ── LEFT column ───────────────────────────────────────────────────

        # miniPC launch
        minipc_box = QGroupBox('MINI PC  ·  remote launch')
        minipc_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        ml = QVBoxLayout(minipc_box)
        delay_row = QHBoxLayout()
        delay_row.addWidget(QLabel('Delay (s):'))
        self.delay_input = QLineEdit('0')
        self.delay_input.setFixedWidth(50)
        self.delay_input.setStyleSheet(
            'background:#0e1018; color:#e8a030; border:1px solid #2a3040;'
            'border-radius:3px; padding:2px 4px;')
        delay_row.addWidget(self.delay_input)
        delay_row.addStretch()
        ml.addLayout(delay_row)
        srow = QHBoxLayout()
        self.minipc_led    = StatusLED('off')
        self.minipc_status = QLabel('not started')
        self.minipc_status.setStyleSheet('color:#506070; font-size:9px;')
        srow.addWidget(self.minipc_led)
        srow.addWidget(self.minipc_status)
        srow.addStretch()
        ml.addLayout(srow)
        self.minipc_btn = self._make_btn(
            'LAUNCH MINI PC  (joy + drive)', '#1a1e10', '#4a6020', '#80aa30')
        self.minipc_btn.clicked.connect(self._launch_minipc)
        ml.addWidget(self.minipc_btn)
        self.cameras_btn = self._make_btn(
            'LAUNCH CAMERAS / NAV', '#101820', '#1a4060', '#2a80c0')
        self.cameras_btn.clicked.connect(self._launch_cameras)
        ml.addWidget(self.cameras_btn)
        left.addWidget(minipc_box)

        rviz_card = ProcessCard('VISUALIZATION  ·  RViz2', self._rviz_cmd)
        rviz_card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        left.addWidget(rviz_card)

        auto_box = QGroupBox('AUTONOMY')
        auto_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        al = QVBoxLayout(auto_box)
        nav_btn  = self._make_btn('Point-Click Navigation', '#101820', '#1a4060', '#2a80c0')
        slam_btn = self._make_btn('SLAM / Mapping',         '#101820', '#1a4060', '#2a80c0')
        nav_btn.clicked.connect(self._start_nav)
        slam_btn.clicked.connect(self._start_slam)
        al.addWidget(nav_btn)
        al.addWidget(slam_btn)
        left.addWidget(auto_box)

        log_box = QGroupBox('SYSTEM LOG')
        log_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        ll = QVBoxLayout(log_box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(60)
        self.log_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        ll.addWidget(self.log_view)
        left.addWidget(log_box, 1)

        stop_all = self._make_btn('⬛  STOP ALL PROCESSES', '#1a0808', '#601010', '#aa2020')
        stop_all.clicked.connect(self._stop_all)
        stop_all.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        left.addWidget(stop_all)

        # ── RIGHT column ──────────────────────────────────────────────────

        # Teleop
        tbox = QGroupBox('TELEOP  ·  TANK DRIVE')
        tbox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        tl = QVBoxLayout(tbox)
        tl.setSpacing(5)

        note = QLabel('🎮  Left stick Y = LEFT wheels  ·  Right stick Y = RIGHT wheels')
        note.setStyleSheet('color:#3a8a50; font-size:9px; padding:2px 0;')
        tl.addWidget(note)

        tr = QHBoxLayout()
        self.teleop_led = StatusLED('off')
        tr.addWidget(self.teleop_led)
        self.teleop_btn = self._make_btn('START TELEOP', '#1d3020', '#3a7a40', '#50aa60')
        self.teleop_btn.setCheckable(True)
        self.teleop_btn.clicked.connect(self._toggle_teleop)
        tr.addWidget(self.teleop_btn, 1)
        self.estop_btn = self._make_btn('E-STOP', '#300d0d', '#a02020', '#ff4040')
        self.estop_btn.clicked.connect(self._emergency_stop)
        tr.addWidget(self.estop_btn)
        tl.addLayout(tr)

        ctrl_info = QLabel(
            'LB=L spd+  LT=L spd-  ·  RB=R spd+  RT=R spd-\n'
            'A=DUMP  Y=DRIVE  B=DIG  X=CAL  ·  D←→=Servo  D↑↓=Act manual')
        ctrl_info.setStyleSheet('color:#3a5060; font-size:9px; padding:2px 0;')
        tl.addWidget(ctrl_info)

        self.dual_speed = DualSpeedWidget()
        self.dual_speed.left_changed.connect(self._left_speed_changed)
        self.dual_speed.right_changed.connect(self._right_speed_changed)
        tl.addWidget(self.dual_speed)
        right.addWidget(tbox)

        # Servo state
        servo_box = QGroupBox('SERVO  ·  D-pad ← CCW  /  D-pad → CW  (360°, hold)')
        servo_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sl = QVBoxLayout(servo_box)
        sl.setSpacing(4)
        self.servo_gauge = ServoGauge()
        self.servo_gauge.setMinimumHeight(70)
        sl.addWidget(self.servo_gauge)
        right.addWidget(servo_box)

        # Actuators
        abox = QGroupBox('ACTUATORS  ·  encoder-based positions + manual hold')
        abox.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        al2 = QVBoxLayout(abox)
        pos_hint = QLabel('A=DUMP  ·  Y=DRIVE  ·  B=DIG  (snap to encoder target)')
        pos_hint.setStyleSheet('color:#405060; font-size:9px;')
        al2.addWidget(pos_hint)
        man_hint = QLabel('D↑ = EXTEND hold  ·  D↓ = RETRACT hold  (fine control)')
        man_hint.setStyleSheet('color:#406050; font-size:9px;')
        al2.addWidget(man_hint)
        abr = QHBoxLayout()
        self.act_dump_btn  = self._make_btn('DUMP  (A)', '#1a2820', '#2a6040', '#40c070')
        self.act_drive_btn = self._make_btn('DRIVE (Y)', '#1a2028', '#2a4060', '#4070c0')
        self.act_dig_btn   = self._make_btn('DIG   (B)', '#281a1a', '#602a2a', '#c04040')
        for b in (self.act_dump_btn, self.act_drive_btn, self.act_dig_btn):
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        abr.addWidget(self.act_dump_btn)
        abr.addWidget(self.act_drive_btn)
        abr.addWidget(self.act_dig_btn)
        al2.addLayout(abr)
        self.act_status = QLabel('Actuator: idle')
        self.act_status.setStyleSheet('color:#607080; font-size:9px;')
        al2.addWidget(self.act_status)
        right.addWidget(abox)

        # Delay simulation
        self.delay_widget = DelayControlWidget(self._toggle_delay_cb)
        right.addWidget(self.delay_widget)

        right.addStretch(1)

        cols.addLayout(left,  45)
        cols.addLayout(right, 55)

        # ── Tab 2: Debug ──────────────────────────────────────────────────
        debug_tab = QWidget()
        debug_tab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tabs.addTab(debug_tab, 'DEBUG')
        debug_lay = QVBoxLayout(debug_tab)
        debug_lay.setContentsMargins(6, 6, 6, 6)
        self.debug_panel = DebugPanel()
        debug_lay.addWidget(self.debug_panel)

        self._log('Mission Control ready  ·  TANK DRIVE mode')
        self._log('D-pad ←→ = Servo  ·  D-pad ↑↓ = Manual actuator (hold)')
        if not ROS_AVAILABLE:
            self._log('⚠  rclpy not found — teleop display disabled')

    # ── Button factory ────────────────────────────────────────────────────

    @staticmethod
    def _make_btn(text, bg, border, hover):
        btn = QPushButton(text)
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btn.setMinimumHeight(28)
        btn.setStyleSheet(f"""
            QPushButton {{
                background:{bg}; color:#a0b8c8;
                border:1px solid {border}; border-radius:5px;
                padding:5px 10px; font-size:10px; font-weight:bold;
                letter-spacing:1px;
            }}
            QPushButton:hover   {{ background:{border}; color:white; }}
            QPushButton:pressed {{ background:{bg}; }}
            QPushButton:checked {{ background:{border}; color:white; }}
        """)
        return btn

    # ── Logging ───────────────────────────────────────────────────────────

    def _log(self, msg):
        self.log_signal.emit(str(msg))

    def _log_direct(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_view.append(
            f"<span style='color:#304050'>[{ts}]</span> {msg}")

    # ── Status subscriber ─────────────────────────────────────────────────

    def _start_status_subscriber(self):
        self._status_thread = JoyStatusSubscriber()
        self._status_thread.status_received.connect(self.debug_panel.update_status)
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
            r  = sp.run(f'ping -c1 -W2 {MINIPC_IP}', shell=True, capture_output=True)
            ok = r.returncode == 0
            self.conn_led.set_color('green' if ok else 'red')
            self.conn_label.setText(
                f'miniPC {MINIPC_IP}: {"online" if ok else "offline"}')
            self._log(f'miniPC {MINIPC_IP}: {"online" if ok else "offline"}')
        threading.Thread(target=run, daemon=True).start()

    # ── MiniPC launch ─────────────────────────────────────────────────────

    def _launch_minipc(self):
        self._log('SSH → miniPC: launching joy_node + joy_to_arduino…')
        self.minipc_led.set_color('yellow')
        self.minipc_status.setText('Starting…')

        remote_script = (
            'source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null\n'
            f'[ -f {MINIPC_WS}/install/setup.bash ] && source {MINIPC_WS}/install/setup.bash\n'
            'export ROS_DOMAIN_ID=42\n'
            'export ROS_LOCALHOST_ONLY=0\n'
            'export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET\n'
            'tmux kill-session -t joy_node 2>/dev/null\n'
            'tmux kill-session -t joy_arduino 2>/dev/null\n'
            'pkill -f joy_to_arduino 2>/dev/null\n'
            'pkill -f joy_node 2>/dev/null\n'
            'sleep 1\n'
            'ROS_SETUP="source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null"\n'
            f'WS_SETUP="[ -f {MINIPC_WS}/install/setup.bash ] && source {MINIPC_WS}/install/setup.bash"\n'
            'ENV_EXPORTS="export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=0; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"\n'
            'tmux new-session -d -s joy_node -x 220 -y 50\n'
            'tmux send-keys -t joy_node "$ROS_SETUP; $WS_SETUP; $ENV_EXPORTS; ros2 run joy joy_node" Enter\n'
            'JOY_UP=false\n'
            'for i in 1 2 3 4 5 6 7 8; do\n'
            '  sleep 1\n'
            '  ros2 topic list 2>/dev/null | grep -q "^/joy$" && JOY_UP=true && echo "[miniPC] /joy live (${i}s)" && break\n'
            'done\n'
            '[ "$JOY_UP" = "false" ] && echo "[miniPC] ✗ joy_node failed" && exit 1\n'
            'tmux new-session -d -s joy_arduino -x 220 -y 50\n'
            f'tmux send-keys -t joy_arduino "$ROS_SETUP; $WS_SETUP; $ENV_EXPORTS; python3 {MINIPC_WS}/joy_to_arduino.py 2>&1 | tee /tmp/rover_arduino.log" Enter\n'
            'sleep 3\n'
            'tmux list-sessions 2>/dev/null | grep -q joy_arduino \\\n'
            '  && echo "[miniPC] ✓ joy_to_arduino running" \\\n'
            '  || echo "[miniPC] ✗ joy_to_arduino session died"\n'
        )
        ssh_cmd = (f'ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=no '
                   f'{MINIPC_USER}@{MINIPC_IP} bash -s')

        def run():
            try:
                proc = subprocess.Popen(ssh_cmd.split(), stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True)
                stdout, _ = proc.communicate(input=remote_script)
                success = False
                for line in stdout.splitlines():
                    self._log(line)
                    if '✓ joy_to_arduino running' in line:
                        success = True
                if success:
                    self._log('✓ miniPC teleop ready')
                    QTimer.singleShot(0, lambda: self.minipc_led.set_color('green'))
                    QTimer.singleShot(0, lambda: self.minipc_status.setText('running'))
                else:
                    QTimer.singleShot(0, lambda: self.minipc_led.set_color('red'))
                    QTimer.singleShot(0, lambda: self.minipc_status.setText('failed'))
            except Exception as e:
                self._log(f'SSH error: {e}')
        threading.Thread(target=run, daemon=True).start()

    def _launch_cameras(self):
        try:
            delay = float(self.delay_input.text() or '0')
        except ValueError:
            delay = 0.0
        self._log(f'SSH → miniPC: launching cameras + nav (delay={delay:.1f}s)…')
        cmd = (f'ssh -o ConnectTimeout=8 -o StrictHostKeyChecking=no '
               f'{MINIPC_USER}@{MINIPC_IP} '
               f'"DELAY_SEC={delay:.1f} nohup bash {MINIPC_WS}/full_launch_minipc.sh '
               f'> /tmp/minipc_launch.log 2>&1 &"')
        def run():
            r = subprocess.run(cmd, shell=True)
            self._log('Cameras launch sent ✓' if r.returncode == 0
                      else 'SSH failed — is miniPC reachable?')
        threading.Thread(target=run, daemon=True).start()

    # ── RViz ─────────────────────────────────────────────────────────────

    def _rviz_cmd(self):
        return (f'rviz2 -d {RVIZ_CONFIG}'
                if os.path.exists(RVIZ_CONFIG) else 'rviz2')

    # ── Autonomy ─────────────────────────────────────────────────────────

    def _start_nav(self):
        self._log('Launching point-click navigation…')
        subprocess.Popen(
            "bash -c 'source /opt/ros/$(ls /opt/ros)/setup.bash && "
            "source ~/lunar_rover_ws/install/setup.bash 2>/dev/null; "
            "ros2 launch lunar_robot_hardware arduino_navigation.launch.py'",
            shell=True)

    def _start_slam(self):
        self._log('Launching SLAM…')
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
        self._teleop_thread.speed_left_changed.connect(
            lambda v: self.dual_speed.set_left(v))
        self._teleop_thread.speed_right_changed.connect(
            lambda v: self.dual_speed.set_right(v))
        self._teleop_thread.servo_state_signal.connect(self._on_ctrl_servo)
        self._teleop_thread.joy_raw_signal.connect(self.debug_panel.update_joy_raw)
        self._teleop_thread.arduino_tx_signal.connect(self.debug_panel.log_tx)
        self._teleop_thread.set_speed_left(self.dual_speed.left_slider.value() / 100.0)
        self._teleop_thread.set_speed_right(self.dual_speed.right_slider.value() / 100.0)
        self._teleop_thread.start()
        self._teleop_active = True
        self.teleop_led.set_color('green')
        self.teleop_btn.setText('STOP TELEOP')
        self._log('Teleop started  ·  GUI is DISPLAY ONLY')
        self._log('D-pad ←→ = Servo  ·  D-pad ↑↓ = Manual actuator hold')

        # Start delay status subscriber
        if self._delay_status_thread is None or not self._delay_status_thread.isRunning():
            self._delay_status_thread = DelayStatusThread()
            self._delay_status_thread.status_received.connect(
                self.delay_widget.on_status_update)
            self._delay_status_thread.start()

    def _stop_teleop(self):
        if self._teleop_thread:
            self._teleop_thread.stop()
            self._teleop_thread = None
        if self._delay_status_thread:
            self._delay_status_thread.stop()
            self._delay_status_thread = None
        self._teleop_active = False
        self.teleop_led.set_color('off')
        self.teleop_btn.setText('START TELEOP')
        self.teleop_btn.setChecked(False)
        self._log('Teleop stopped')

    def _emergency_stop(self):
        if self._teleop_thread:
            self._teleop_thread.emergency_stop()
        self._log('⬛ E-STOP')
        self.debug_panel.log_tx('[GUI] E-STOP button pressed')

    # ── Speed ─────────────────────────────────────────────────────────────

    def _left_speed_changed(self, val: float):
        if self._teleop_thread:
            self._teleop_thread.set_speed_left(val)

    def _right_speed_changed(self, val: float):
        if self._teleop_thread:
            self._teleop_thread.set_speed_right(val)

    # ── Servo ─────────────────────────────────────────────────────────────

    def _on_ctrl_servo(self, state: str):
        self.servo_gauge.set_state(state)

    # ── Delay simulation ──────────────────────────────────────────────────

    def _toggle_delay_cb(self, enabled: bool):
        """Toggle 5-second comms delay: publishes /delay_enabled to miniPC."""
        self._delay_enabled = enabled
        state = 'ENABLED' if enabled else 'DISABLED'
        self._log(f'Communication delay {state}')

        if ROS_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                if self._delay_pub_node is None:
                    self._delay_pub_node = rclpy.create_node('rover_delay_pub')
                    self._delay_ros_pub  = self._delay_pub_node.create_publisher(
                        Bool, '/delay_enabled', 10)
                m = Bool(); m.data = enabled
                self._delay_ros_pub.publish(m)
            except Exception as e:
                self._log(f'Delay ROS publish error: {e}')

        # Restart camera pipelines on miniPC with matching delay value
        delay_val = '5.0' if enabled else '0.0'
        self._restart_pipelines_with_delay(delay_val)

    def _restart_pipelines_with_delay(self, delay_sec: str):
        """SSH to miniPC and restart image pipelines with new buffer_delay_sec."""
        self._log(f'Restarting camera pipelines with delay={delay_sec}s…')
        script = (
            'source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null\n'
            f'[ -f {MINIPC_WS}/install/setup.bash ] && source {MINIPC_WS}/install/setup.bash\n'
            'export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=0\n'
            'export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET\n'
            'pkill -f "optimized_image_pipeline" 2>/dev/null\n'
            'sleep 1\n'
            f'python3 {MINIPC_WS}/optimized_image_pipeline.py --ros-args '
            f'-p input_topic:=/camera/camera/color/image_raw '
            f'-p output_topic:=/camera/color/stream/compressed '
            f'-p input_is_compressed:=false -p input_reliable:=false '
            f'-p jpeg_quality:=30 -p decimation:=5 '
            f'-p buffer_delay_sec:={delay_sec} '
            f'-p target_fps:=6.0 > /tmp/rover_pipe_color.log 2>&1 &\n'
            f'python3 {MINIPC_WS}/optimized_image_pipeline.py --ros-args '
            f'-p input_topic:=/camera/camera/aligned_depth_to_color/image_raw '
            f'-p output_topic:=/camera/depth/stream/compressed '
            f'-p input_is_compressed:=false -p input_reliable:=false '
            f'-p jpeg_quality:=50 -p decimation:=10 '
            f'-p buffer_delay_sec:={delay_sec} '
            f'-p target_fps:=3.0 > /tmp/rover_pipe_depth.log 2>&1 &\n'
            'echo "[miniPC] pipelines restarted"\n'
        )
        def run():
            try:
                proc = subprocess.Popen(
                    [f'ssh', '-o', 'ConnectTimeout=8',
                     '-o', 'StrictHostKeyChecking=no',
                     f'{MINIPC_USER}@{MINIPC_IP}', 'bash', '-s'],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True)
                stdout, _ = proc.communicate(input=script, timeout=20)
                for line in stdout.splitlines():
                    self._log(line)
            except Exception as e:
                self._log(f'Pipeline restart error: {e}')
        threading.Thread(target=run, daemon=True).start()

    # ── Stop all ─────────────────────────────────────────────────────────

    def _stop_all(self):
        self._stop_teleop()
        self._log('Stopping all processes…')
        subprocess.run('pkill -f rviz2; pkill -f unified_navigator', shell=True)

    # ── Cleanup ───────────────────────────────────────────────────────────

    def closeEvent(self, event):
        if self._status_thread:
            self._status_thread.stop()
        if self._delay_status_thread:
            self._delay_status_thread.stop()
        super().closeEvent(event)


# ═════════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setFont(QFont('Courier New', 10))
    w = MissionControl()
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()