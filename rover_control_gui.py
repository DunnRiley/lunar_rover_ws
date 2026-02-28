#!/usr/bin/env python3
"""
Lunar Rover Mission Control GUI  —  rover_control_gui.py

FIXES vs previous version:
  - Forward/backward and turn axes corrected for mirrored motor mounts
  - Arc-turn slider: blend between spin-in-place (0) and straight (100)
  - Delay input field now correctly passes value to SSH launch command
  - Stopping teleop also kills joy_node and teleop subprocess so no stale signals
  - E-stop sends /emergency_stop=True then False (clear) on resume
"""

import os, sys, math, subprocess, threading, time, signal

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit, QSlider,
    QSizePolicy, QFrame, QLineEdit
)
from PyQt5.QtGui  import QFont, QColor, QPainter, QBrush, QPen
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int8 as RosInt8, Bool as RosBool
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ── CONFIG ────────────────────────────────────────────────────────────────
MINIPC_USER = "cheese"
MINIPC_IP   = "192.168.0.102"
MINIPC_WS   = "~/lunar_rover_ws"
RVIZ_CONFIG = os.path.expanduser("~/lunar_rover_ws/laptop_stream.rviz")

# ── Motor inversion (must match joy_to_arduino.py) ────────────────────────
INVERT_FWD  = True   # Flip if rover drives backwards when pushing forward
INVERT_TURN = False  # Flip if turning goes the wrong way

# ═════════════════════════════════════════════════════════════════════════
# TELEOP PUBLISHER — rclpy node in a QThread
# ═════════════════════════════════════════════════════════════════════════
class TeleopPublisher(QThread):
    status_changed = pyqtSignal(str)
    speed_changed  = pyqtSignal(float)
    arc_changed    = pyqtSignal(float)

    # Controller mapping
    JOY_AXIS_FWD  = 1    # Left  stick Y  (+1 = forward on most pads)
    JOY_AXIS_TURN = 3    # Right stick X  (+1 = left)
    JOY_BTN_LB    = 4    # Left  bumper → actuator EXTEND
    JOY_BTN_RB    = 5    # Right bumper → actuator RETRACT
    JOY_BTN_X     = 2    # X button     → speed UP
    JOY_BTN_B     = 1    # B button     → speed DOWN
    JOY_BTN_START = 7    # Start        → e-stop toggle
    JOY_DEADZONE  = 0.10
    SPEED_STEP    = 0.05

    def __init__(self):
        super().__init__()
        self._lock    = threading.Lock()
        self._running = False
        self._speed   = 0.5
        self._arc     = 1.0   # 1.0 = full differential (spin), 0.0 = straight only
        self._node    = None
        self._pub     = None
        self._act_pub = None
        self._estop_pub = None
        self._emergency = False

        self._want_lin = 0.0
        self._want_ang = 0.0
        self._want_act = 0

        self._sent_lin = None
        self._sent_ang = None
        self._sent_act = None

        self._prev_btns = {}

    def set_speed(self, v: float):
        with self._lock:
            self._speed = max(0.05, min(1.0, v))

    def set_arc(self, v: float):
        """v in [0.0, 1.0].  1.0 = maximum arc (full differential).
           0.0 = near-straight (minimal turn component)."""
        with self._lock:
            self._arc = max(0.0, min(1.0, v))

    def emergency_stop(self):
        with self._lock:
            self._want_lin = 0.0
            self._want_ang = 0.0
            self._want_act = 0
        self._emergency = True
        if self._estop_pub:
            try:
                m = RosBool(); m.data = True
                self._estop_pub.publish(m)
            except Exception:
                pass

    def clear_estop(self):
        self._emergency = False
        if self._estop_pub:
            try:
                m = RosBool(); m.data = False
                self._estop_pub.publish(m)
            except Exception:
                pass

    def send_actuator(self, value: int):
        with self._lock:
            self._want_act = value

    # ── Joy helpers ───────────────────────────────────────────────────────

    def _dz(self, v):
        return v if abs(v) >= self.JOY_DEADZONE else 0.0

    def _rising(self, idx, cur):
        prev = self._prev_btns.get(idx, 0)
        self._prev_btns[idx] = cur
        return cur == 1 and prev == 0

    def _joy_cb(self, msg):
        try:
            ax  = lambda i: msg.axes[i]    if i < len(msg.axes)    else 0.0
            btn = lambda i: msg.buttons[i] if i < len(msg.buttons) else 0

            # E-stop toggle
            if self._rising(self.JOY_BTN_START, btn(self.JOY_BTN_START)):
                if self._emergency:
                    self.clear_estop()
                    self.status_changed.emit('E-stop cleared via controller')
                else:
                    self.emergency_stop()
                    self.status_changed.emit('⬛ E-STOP (controller)')

            if self._emergency:
                for b in (self.JOY_BTN_LB, self.JOY_BTN_RB,
                          self.JOY_BTN_X,  self.JOY_BTN_B):
                    self._prev_btns[b] = btn(b)
                return

            # Speed adjust
            if self._rising(self.JOY_BTN_X, btn(self.JOY_BTN_X)):
                with self._lock:
                    self._speed = round(min(1.0, self._speed + self.SPEED_STEP), 2)
                    s = self._speed
                self.speed_changed.emit(s)
                self.status_changed.emit(f"Speed: {s:.2f}")

            if self._rising(self.JOY_BTN_B, btn(self.JOY_BTN_B)):
                with self._lock:
                    self._speed = round(max(0.05, self._speed - self.SPEED_STEP), 2)
                    s = self._speed
                self.speed_changed.emit(s)
                self.status_changed.emit(f"Speed: {s:.2f}")

            # Actuators
            lb  = btn(self.JOY_BTN_LB)
            rb  = btn(self.JOY_BTN_RB)
            act = 1 if lb else (-1 if rb else 0)

            # Drive axes — apply inversion to match physical rover orientation
            fwd  = self._dz(ax(self.JOY_AXIS_FWD))
            turn = self._dz(ax(self.JOY_AXIS_TURN))

            if INVERT_FWD:
                fwd = -fwd
            if INVERT_TURN:
                turn = -turn

            with self._lock:
                spd = self._speed
                arc = self._arc

            # Arc scaling: arc=1.0 → full angular_scale, arc=0.0 → 0.1 (near straight)
            angular_scale = 0.1 + arc * 1.1   # range [0.1 … 1.2]
            lin = fwd  * spd
            ang = turn * spd * angular_scale

            with self._lock:
                self._want_lin = lin
                self._want_ang = ang
                self._want_act = act

        except Exception as e:
            self.status_changed.emit(f"Joy error: {e}")

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
            self.status_changed.emit("ROS2 not available")
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
            self._pub      = self._node.create_publisher(Twist,   '/cmd_vel',        qos)
            self._act_pub  = self._node.create_publisher(RosInt8, '/actuator_cmd',   qos)
            self._estop_pub = self._node.create_publisher(RosBool, '/emergency_stop', 10)

            try:
                from sensor_msgs.msg import Joy
                self._node.create_subscription(Joy, '/joy', self._joy_cb, 10)
                self.status_changed.emit("Teleop active  ·  awaiting controller")
            except Exception:
                self.status_changed.emit("Teleop active  ·  no sensor_msgs")

            self._running = True
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._node)

            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.02)
                self._flush()

        except Exception as e:
            self.status_changed.emit(f"Teleop error: {e}")
        finally:
            self._running = False
            if self._node:
                try:
                    # Send stop before destroying
                    stop = Twist()
                    self._pub.publish(stop)
                except Exception:
                    pass
                try:
                    self._node.destroy_node()
                except Exception:
                    pass

    def stop(self):
        # Send zero velocity before stopping
        with self._lock:
            self._want_lin = 0.0
            self._want_ang = 0.0
            self._want_act = 0
        self._running = False
        self.quit()
        self.wait(3000)


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
        if checked:
            self._start()
        else:
            self._stop()

    def _start(self):
        cmd = self._cmd_fn()
        if not cmd:
            self.btn.setChecked(False)
            return
        self._proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
        self.led.set_color("green")
        self.btn.setText("STOP")
        self.log.setText(f"PID {self._proc.pid}")

    def _stop(self):
        if self._proc:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            except Exception:
                self._proc.terminate()
            self._proc = None
        self.led.set_color("off")
        self.btn.setText("START")
        self.btn.setChecked(False)
        self.log.setText("stopped")


# ═════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ═════════════════════════════════════════════════════════════════════════
class MissionControl(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lunar Rover Mission Control")
        self.setMinimumWidth(860)
        self.setMinimumHeight(700)

        self._teleop_active  = False
        self._teleop_thread  = None
        self._joy_proc       = None   # joy_node subprocess
        self._teleop_proc    = None   # arduino_teleop_controller subprocess

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
            QSlider::groove:horizontal {
                background:#1a1e28; height:6px; border-radius:3px;
            }
            QSlider::handle:horizontal {
                background:#e8a030; width:14px; height:14px;
                margin:-4px 0; border-radius:7px;
            }
            QSlider::sub-page:horizontal { background:#3a6040; border-radius:3px; }
            QLabel { color:#c0cce0; font-size:10px; }
            QLineEdit {
                background:#0e1018; color:#e8a030;
                border:1px solid #2a3040; border-radius:3px;
                padding:2px 4px;
            }
        """)

    # ── UI ────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 12)

        # Header
        hdr = QLabel("⬡  LUNAR ROVER  ·  MISSION CONTROL")
        hdr.setStyleSheet("color:#e8a030; font-size:14px; font-weight:bold; letter-spacing:3px; padding:4px 0;")
        root.addWidget(hdr)

        # Connection bar
        cbar = QHBoxLayout()
        self.conn_led    = StatusLED("off")
        self.conn_label  = QLabel("miniPC: checking…")
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
        cols.addLayout(left,  55)
        cols.addLayout(right, 45)
        root.addLayout(cols)

        # ── LEFT: miniPC ──────────────────────────────────────────────────
        minipc_box = QGroupBox("MINI PC  ·  remote launch")
        ml = QVBoxLayout(minipc_box)

        # Delay row
        delay_row = QHBoxLayout()
        delay_row.addWidget(QLabel("Delay (s):"))
        self.delay_input = QLineEdit("0")
        self.delay_input.setFixedWidth(55)
        delay_row.addWidget(self.delay_input)
        delay_row.addStretch()
        ml.addLayout(delay_row)

        # Status row
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
        slam_btn = self._make_btn("SLAM / Mapping",        "#101820", "#1a4060", "#2a80c0")
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

        # Stop all button
        stop_all = self._make_btn("⬛  STOP ALL PROCESSES", "#1a0808", "#601010", "#aa2020")
        stop_all.clicked.connect(self._stop_all)
        left.addWidget(stop_all)

        # ── RIGHT: Teleop ─────────────────────────────────────────────────
        tbox = QGroupBox("TELEOP  ·  gamepad")
        tl = QVBoxLayout(tbox)
        tl.setSpacing(6)

        note = QLabel("🎮  Gamepad → /joy  (motors corrected for mirrored mount)")
        note.setStyleSheet("color:#3a8a50; font-size:9px; padding:2px 0;")
        tl.addWidget(note)

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
            "Left stick = drive  ·  Right stick = turn\n"
            "LB = extend  ·  RB = retract\n"
            "X = speed+  ·  B = speed−  ·  Start = e-stop"
        )
        ctrl_info.setStyleSheet("color:#3a5060; font-size:9px; padding:2px 0;")
        tl.addWidget(ctrl_info)

        # ── Speed slider ──────────────────────────────────────────────────
        sr = QHBoxLayout()
        sr.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(5, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self._speed_changed)
        sr.addWidget(self.speed_slider)
        self.speed_label = QLabel("0.50")
        self.speed_label.setStyleSheet("color:#e8a030; min-width:38px;")
        sr.addWidget(self.speed_label)
        tl.addLayout(sr)

        # ── Arc-turn slider ───────────────────────────────────────────────
        arc_header = QLabel("ARC TURN  ·  ratio of inner to outer wheel speed")
        arc_header.setStyleSheet("color:#6080a0; font-size:9px; font-weight:bold; letter-spacing:1px; padding-top:4px;")
        tl.addWidget(arc_header)

        arc_desc = QLabel(
            "LEFT  = tight spin (0% inner wheel)\n"
            "RIGHT = gentle arc (inner wheel matches outer)\n"
            "Tip: lower arc = tank turn, higher arc = sweeping curve"
        )
        arc_desc.setStyleSheet("color:#3a5060; font-size:9px;")
        tl.addWidget(arc_desc)

        ar = QHBoxLayout()
        ar.addWidget(QLabel("Tight"))
        self.arc_slider = QSlider(Qt.Horizontal)
        self.arc_slider.setRange(0, 100)
        self.arc_slider.setValue(50)          # default: 50% arc
        self.arc_slider.setStyleSheet("""
            QSlider::groove:horizontal { background:#1a1e28; height:6px; border-radius:3px; }
            QSlider::handle:horizontal {
                background:#60a8e0; width:14px; height:14px;
                margin:-4px 0; border-radius:7px;
            }
            QSlider::sub-page:horizontal { background:#204060; border-radius:3px; }
        """)
        self.arc_slider.valueChanged.connect(self._arc_changed)
        ar.addWidget(self.arc_slider)
        ar.addWidget(QLabel("Wide"))

        # Inner-wheel % display
        self.arc_label = QLabel("inner: 50%")
        self.arc_label.setStyleSheet("color:#60a8e0; min-width:70px;")
        ar.addWidget(self.arc_label)
        tl.addLayout(ar)

        # Outer / inner wheel % readout
        self.arc_detail = QLabel("outer: 100%  inner: 50%")
        self.arc_detail.setStyleSheet("color:#304050; font-size:9px;")
        tl.addWidget(self.arc_detail)

        right.addWidget(tbox)

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

        self._log("Mission Control ready.")
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
                padding:6px 12px; font-size:10px; font-weight:bold; letter-spacing:1px;
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
        self.log_view.append(f"<span style='color:#304050'>[{ts}]</span> {msg}")

    # ── Connection checker ────────────────────────────────────────────────
    def _start_connection_checker(self):
        self._conn_timer = QTimer()
        self._conn_timer.timeout.connect(self._check_connection)
        self._conn_timer.start(6000)
        self._check_connection()

    def _check_connection(self):
        def run():
            r = subprocess.run(f"ping -c1 -W2 {MINIPC_IP}", shell=True,
                               capture_output=True)
            ok = r.returncode == 0
            self.conn_led.set_color("green" if ok else "red")
            self.conn_label.setText(f"miniPC {MINIPC_IP}: {'online' if ok else 'offline'}")
        threading.Thread(target=run, daemon=True).start()

    # ── MiniPC launch ─────────────────────────────────────────────────────
    def _launch_minipc(self):
        # --- FIX: read delay from input field and validate it ---
        raw = self.delay_input.text().strip()
        try:
            delay = float(raw) if raw else 0.0
            if delay < 0:
                delay = 0.0
        except ValueError:
            self._log(f"⚠  Invalid delay '{raw}' — using 0.0")
            delay = 0.0
            self.delay_input.setText("0")

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
                self._log(f"MiniPC launch sent ✓  (delay={delay_str}s)")
                self.minipc_led.set_color("green")
                self.minipc_status.setText(f"Launched  ·  {mode}")
            else:
                self._log("SSH failed — is miniPC reachable?")
                self.minipc_led.set_color("red")
                self.minipc_status.setText("SSH failed")
        threading.Thread(target=run, daemon=True).start()

    # ── RViz ─────────────────────────────────────────────────────────────
    def _rviz_cmd(self):
        # RViz MUST be launched inside a properly sourced ROS environment,
        # otherwise ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY are wrong and no
        # topics from the miniPC are discovered.
        config_arg = f"-d {RVIZ_CONFIG}" if os.path.exists(RVIZ_CONFIG) else ""
        ros_distro = None
        for d in ["/opt/ros/jazzy", "/opt/ros/humble", "/opt/ros/iron"]:
            if os.path.exists(d + "/setup.bash"):
                ros_distro = d
                break
        if ros_distro is None:
            self._log("⚠  ROS2 not found at /opt/ros — RViz may not work")
            return f"rviz2 {config_arg}".strip()

        ws_setup = os.path.expanduser("~/lunar_rover_ws/install/setup.bash")
        source_ws = f"[ -f {ws_setup} ] && source {ws_setup};" if os.path.exists(ws_setup) else ""
        return (
            f"bash -c '"
            f"source {ros_distro}/setup.bash; "
            f"{source_ws} "
            f"export ROS_DOMAIN_ID=42; "
            f"export ROS_LOCALHOST_ONLY=0; "
            f"export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; "
            f"rviz2 {config_arg}'"
        ).strip()

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
        if checked:
            self._start_teleop()
        else:
            self._stop_teleop()

    def _start_teleop(self):
        if self._teleop_thread and self._teleop_thread.isRunning():
            return

        # Start joy_node as a subprocess so we can kill it later
        self._log("Starting joy_node (USB gamepad → /joy)…")
        try:
            self._joy_proc = subprocess.Popen(
                "bash -c 'source /opt/ros/$(ls /opt/ros | head -1)/setup.bash && "
                "ros2 run joy joy_node'",
                shell=True,
                preexec_fn=os.setsid  # put in own process group for clean kill
            )
            self._log(f"joy_node started (PID {self._joy_proc.pid})")
        except Exception as e:
            self._log(f"⚠ joy_node failed to start: {e}")
            self._joy_proc = None

        self._teleop_thread = TeleopPublisher()
        self._teleop_thread.status_changed.connect(self._log)
        self._teleop_thread.speed_changed.connect(self._on_ctrl_speed)
        self._teleop_thread.start()
        self._teleop_active = True
        self.teleop_led.set_color("green")
        self.teleop_btn.setText("STOP TELEOP")

        # Sync arc slider to thread
        arc_val = self.arc_slider.value() / 100.0
        self._teleop_thread.set_arc(arc_val)

        self._log("Teleop started — plug in controller")

    def _stop_teleop(self):
        """Stop teleop thread AND kill joy_node so no stale signals remain."""
        # 1. Stop the ROS publisher thread (sends zero vel first)
        if self._teleop_thread:
            self._teleop_thread.stop()
            self._teleop_thread = None

        # 2. Kill joy_node process group
        if self._joy_proc:
            try:
                os.killpg(os.getpgid(self._joy_proc.pid), signal.SIGTERM)
                self._log("joy_node stopped")
            except Exception as e:
                self._log(f"joy_node kill error (may already be dead): {e}")
            self._joy_proc = None

        # 3. Belt-and-suspenders: pkill any stray joy_node
        subprocess.run("pkill -f 'joy_node'", shell=True, capture_output=True)

        self._teleop_active = False
        self.teleop_led.set_color("off")
        self.teleop_btn.setText("START TELEOP")
        self.teleop_btn.setChecked(False)
        self._log("Teleop stopped — all signals killed")

    def _emergency_stop(self):
        if self._teleop_thread:
            self._teleop_thread.emergency_stop()
        self._log("⬛ E-STOP")

    def _on_ctrl_speed(self, spd: float):
        self.speed_slider.blockSignals(True)
        self.speed_slider.setValue(int(spd * 100))
        self.speed_slider.blockSignals(False)
        self.speed_label.setText(f"{spd:.2f}")

    def _speed_changed(self, val):
        spd = val / 100.0
        self.speed_label.setText(f"{spd:.2f}")
        if self._teleop_thread:
            self._teleop_thread.set_speed(spd)

    # ── Arc-turn slider ───────────────────────────────────────────────────
    def _arc_changed(self, val):
        """val: 0–100.  0 = tight spin (inner wheel 0%), 100 = wide arc (inner ~90%)."""
        # Inner wheel percentage: 0% at val=0, ~90% at val=100
        inner_pct = int(val * 0.9)
        self.arc_label.setText(f"inner: {inner_pct}%")
        self.arc_detail.setText(f"outer: 100%   inner: {inner_pct}%   ratio: {val}%")

        # Send to teleop thread as 0.0–1.0
        arc_f = val / 100.0
        if self._teleop_thread:
            self._teleop_thread.set_arc(arc_f)

    # ── Actuator GUI buttons ──────────────────────────────────────────────
    def _act_gui(self, value: int):
        if self._teleop_thread:
            self._teleop_thread.send_actuator(value)
        labels = {1: "▲ Extending…", -1: "▼ Retracting…", 0: "Actuator: idle"}
        colors = {1: "#40c070",       -1: "#c04040",        0: "#607080"}
        self.act_status.setText(labels.get(value, ""))
        self.act_status.setStyleSheet(f"color:{colors.get(value,'#607080')}; font-size:9px;")

    # ── Stop all ─────────────────────────────────────────────────────────
    def _stop_all(self):
        self._stop_teleop()
        self._log("Stopping all processes…")
        subprocess.run("pkill -f rviz2; pkill -f unified_navigator; pkill -f slam_minipc",
                       shell=True)

    # ── Clean shutdown ────────────────────────────────────────────────────
    def closeEvent(self, event):
        self._stop_teleop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("Courier New", 10))
    w = MissionControl()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()