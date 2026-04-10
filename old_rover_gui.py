#!/usr/bin/env python3
"""
rover_mission_gui.py  —  Lunar Rover Mission Control
Run on the LAPTOP:  python3 ~/lunar_rover_ws/rover_mission_gui.py
"""

import math, os, sys, glob, json, subprocess, threading, time
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit, QSizePolicy, QFrame,
    QLineEdit, QTabWidget, QFileDialog, QScrollArea, QComboBox,
    QDoubleSpinBox, QSpinBox, QCheckBox, QProgressBar, QSplitter, QSlider,
)
from PyQt5.QtGui  import QFont, QColor, QPainter, QBrush, QPen
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, pyqtSlot

try:
    import rclpy
    from std_msgs.msg import Bool, String, Float32, Float32MultiArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# ─── Config ───────────────────────────────────────────────────────────────────
MINIPC_USER = "cheese"
MINIPC_IP   = "192.168.0.102"
MINIPC_WS   = "~/lunar_rover_ws"          # tilde-style, expanded by the miniPC
RVIZ_CONFIG = os.path.expanduser("~/lunar_rover_ws/laptop_stream.rviz")

ACTION_DEFAULTS = {
    "drive_forward":     {"distance_m": 1.0,  "speed": 120, "timeout_s": 120},
    "drive_backward":    {"distance_m": 0.5,  "speed": 120, "timeout_s": 120},
    "pivot_turn":        {"degrees":    90.0,  "speed": 100, "timeout_s": 60},
    "actuator_position": {"target":    "dig",  "timeout_s": 20},
    "wait":              {"seconds":    1.0},
    "stop":              {},
}

# ─── Button factory ───────────────────────────────────────────────────────────
# Defined ONCE here so every part of the file can use it safely.

class _Btn(QPushButton):
    """QPushButton with a fluent .connect() helper."""
    def on_click(self, fn):
        self.clicked.connect(fn)
        return self


def btn(text, bg, border, hover, h=34, w=None) -> _Btn:
    b = _Btn(text)
    b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
    b.setMinimumHeight(h)
    if w:
        b.setMinimumWidth(w)
    b.setStyleSheet(
        f"QPushButton{{background:{bg};color:#b0c8d8;border:1px solid {border};"
        f"border-radius:5px;padding:3px 10px;font-size:13px;font-weight:bold;}}"
        f"QPushButton:hover{{background:{border};color:#fff;}}"
        f"QPushButton:pressed{{background:{bg};}}"
    )
    return b


# ─── LED indicator ────────────────────────────────────────────────────────────

class LED(QLabel):
    _C = {"off": QColor(50,50,60), "green": QColor(50,210,80),
          "yellow": QColor(240,190,40), "red": QColor(210,60,60)}
    def __init__(self, c="off", parent=None):
        super().__init__(parent); self.setFixedSize(14, 14); self._c = c
    def set_color(self, c): self._c = c; self.update()
    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        c = self._C.get(self._c, self._C["off"])
        p.setBrush(QBrush(c)); p.setPen(QPen(c.darker(160), 1))
        p.drawEllipse(1, 1, 12, 12)


# ─── ROS monitor thread ───────────────────────────────────────────────────────

class RosMonitor(QThread):
    mission_status = pyqtSignal(dict)
    def __init__(self):
        super().__init__(); self._running = False
    def run(self):
        if not ROS_AVAILABLE: return
        try:
            if not rclpy.ok(): rclpy.init()
            node = rclpy.create_node("rover_gui_monitor")
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=5)
            def _cb(msg):
                try: self.mission_status.emit(json.loads(msg.data))
                except: pass
            node.create_subscription(String, "/mission/status", _cb, q)
            self._running = True
            from rclpy.executors import SingleThreadedExecutor
            ex = SingleThreadedExecutor(); ex.add_node(node)
            while self._running and rclpy.ok():
                ex.spin_once(timeout_sec=0.05)
        except Exception as e:
            print(f"[RosMonitor] {e}")
    def stop(self):
        self._running = False; self.quit(); self.wait(2000)


# ─── Mission step widget ──────────────────────────────────────────────────────

class StepWidget(QFrame):
    sig_remove = pyqtSignal(object)
    sig_up     = pyqtSignal(object)
    sig_down   = pyqtSignal(object)

    def __init__(self, action, params, parent=None):
        super().__init__(parent)
        self.action = action; self.params = dict(params)
        self.setStyleSheet(
            "QFrame{background:#141820;border:1px solid #2a3848;"
            "border-radius:5px;margin:1px;}")
        root = QVBoxLayout(self); root.setSpacing(2); root.setContentsMargins(8,5,8,5)
        hr = QHBoxLayout()
        lbl = QLabel(action.replace("_", " ").upper())
        lbl.setStyleSheet("color:#80b8d8;font-size:13px;font-weight:bold;")
        hr.addWidget(lbl); hr.addStretch()
        for sym, sig in [("^", self.sig_up), ("v", self.sig_down)]:
            b2 = QPushButton(sym); b2.setFixedSize(24, 24)
            b2.setStyleSheet("background:#0e1018;color:#507080;border:1px solid #1c3040;"
                             "border-radius:3px;font-size:11px;font-weight:bold;")
            b2.clicked.connect(lambda _, s=sig: s.emit(self))
            hr.addWidget(b2)
        xb = QPushButton("X"); xb.setFixedSize(24, 24)
        xb.setStyleSheet("background:#1a0808;color:#c05050;border:1px solid #401010;"
                         "border-radius:3px;font-size:11px;font-weight:bold;")
        xb.clicked.connect(lambda: self.sig_remove.emit(self))
        hr.addWidget(xb)
        root.addLayout(hr)
        if params:
            pl = QLabel("   ".join(f"{k}={v}" for k, v in params.items()))
            pl.setStyleSheet("color:#607890;font-size:12px;")
            pl.setWordWrap(True); root.addWidget(pl)

    def to_dict(self):
        return {"action": self.action, "params": self.params}


# ─── Main window ──────────────────────────────────────────────────────────────

class GUI(QMainWindow):
    _log_sig = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._log_sig.connect(self._log_direct)
        self.setWindowTitle("Lunar Rover — Mission Control")
        self.resize(1200, 900)
        self.setMinimumSize(960, 720)

        self._steps      = []
        self._yaml_path  = ""
        self._ros_pub_nd = None
        self._start_pub  = self._file_pub  = None
        self._dist_pub   = self._cmd_pub   = self._turn_pub = None

        self.setStyleSheet("""
            QMainWindow,QWidget{background:#0c0f16;color:#c8d8e8;
                font-family:'Courier New',monospace;font-size:13px;}
            QGroupBox{border:1px solid #2a3848;border-radius:6px;
                margin-top:12px;padding:8px;color:#5888a8;
                font-size:12px;font-weight:bold;letter-spacing:1px;}
            QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 5px;}
            QTextEdit{background:#080b10;color:#6888a0;border:1px solid #182030;
                border-radius:4px;font-family:monospace;font-size:12px;}
            QTabWidget::pane{border:1px solid #2a3848;background:#0c0f16;}
            QTabBar::tab{background:#0c0f16;color:#506878;border:1px solid #2a3848;
                border-bottom:none;padding:7px 20px;font-size:13px;}
            QTabBar::tab:selected{background:#141820;color:#c8d8e8;}
            QComboBox{background:#0e1218;color:#c0d0e0;border:1px solid #1c3040;
                border-radius:4px;padding:3px 8px;font-size:13px;min-height:28px;}
            QComboBox QAbstractItemView{background:#0e1218;color:#c0d0e0;
                selection-background-color:#1a3040;}
            QDoubleSpinBox,QSpinBox{background:#0e1218;color:#c0d0e0;
                border:1px solid #1c3040;border-radius:4px;
                font-size:13px;min-height:26px;padding:2px;}
            QLineEdit{background:#0e1218;color:#c0d0e0;border:1px solid #1c3040;
                border-radius:4px;padding:3px 8px;font-size:13px;min-height:26px;}
            QCheckBox{color:#a0b8c8;font-size:13px;}
            QLabel{font-size:13px;}
            QScrollArea{background:#080b10;}
            QScrollBar:vertical{background:#0c0f16;width:10px;}
            QScrollBar::handle:vertical{background:#2a3848;border-radius:5px;}
            QSlider::groove:horizontal{background:#1a2030;height:6px;border-radius:3px;}
            QSlider::handle:horizontal{background:#4080b0;width:16px;height:16px;
                margin:-5px 0;border-radius:8px;}
            QSlider::sub-page:horizontal{background:#2a5080;border-radius:3px;}
        """)

        cw = QWidget(); self.setCentralWidget(cw)
        root = QVBoxLayout(cw); root.setSpacing(6); root.setContentsMargins(10,10,10,10)

        # Header
        hrow = QHBoxLayout()
        ttl = QLabel("LUNAR ROVER  —  MISSION CONTROL")
        ttl.setStyleSheet("color:#e8a030;font-size:18px;font-weight:bold;letter-spacing:3px;")
        hrow.addWidget(ttl); hrow.addStretch()
        self._conn_led = LED("off")
        self._conn_lbl = QLabel(f"miniPC {MINIPC_IP}: checking...")
        self._conn_lbl.setStyleSheet("color:#506070;font-size:12px;")
        hrow.addWidget(self._conn_led); hrow.addWidget(self._conn_lbl)
        root.addLayout(hrow)

        tabs = QTabWidget()
        tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(tabs, 1)
        tabs.addTab(self._tab_control(), "CONTROL")
        tabs.addTab(self._tab_mission(), "MISSION")
        tabs.addTab(self._tab_log(),     "LOG")

        self._start_ros_monitor()
        self._start_conn_check()

    # ═════════════════════════════════════════════════════════════════════════
    # CONTROL TAB
    # ═════════════════════════════════════════════════════════════════════════

    def _tab_control(self):
        w = QWidget()
        root = QVBoxLayout(w); root.setSpacing(8); root.setContentsMargins(10,10,10,10)

        # ── TELEOP ────────────────────────────────────────────────────────────
        tg = QGroupBox("TELEOP  —  joystick via miniPC")
        tv = QVBoxLayout(tg); tv.setSpacing(5)
        tv.addWidget(QLabel(
            "L-stick Y = left wheels    R-stick Y = right wheels\n"
            "A=DUMP  Y=DRIVE  B=DIG  X=CAL    D-pad UP/DOWN=actuator hold    D-pad LR=servo hold\n"
            "LB=L speed+    LT=L speed−    RB=R speed+    RT=R speed−    Start=e-stop toggle"))
        joy_row = QHBoxLayout()
        self._joy_led = LED("off")
        self._joy_lbl = QLabel("not running")
        self._joy_lbl.setStyleSheet("color:#506070;font-size:12px;")
        joy_row.addWidget(self._joy_led); joy_row.addWidget(self._joy_lbl, 1)
        joy_row.addWidget(
            btn("START TELEOP","#081808","#1a5020","#28a040",h=40,w=200)
            .on_click(self._start_teleop))
        joy_row.addWidget(
            btn("STOP TELEOP","#1a0808","#501010","#a02020",h=40,w=140)
            .on_click(self._stop_teleop))
        tv.addLayout(joy_row)
        root.addWidget(tg)

        # ── STRAIGHT DRIVE ────────────────────────────────────────────────────
        dg = QGroupBox("STRAIGHT DRIVE  —  via /nav/arduino_dist_cmd → nav_arduino_bridge")
        dv = QVBoxLayout(dg); dv.setSpacing(5)
        dr = QHBoxLayout()
        for lbl, m in [("0.5m FWD",0.5),("1m FWD",1.0),("2m FWD",2.0),
                       ("0.5m REV",-0.5),("1m REV",-1.0)]:
            dr.addWidget(
                btn(lbl,"#0e1018","#1c3848","#2a6060",h=36,w=90)
                .on_click(lambda _,v=m: self._send_dist(v)))
        dr.addStretch()
        dr.addWidget(
            btn("STOP ALL","#1a0808","#601010","#c02020",h=36,w=110)
            .on_click(lambda: self._send_raw_cmd(0xFF,0,0,0)))
        dv.addLayout(dr)

        cr = QHBoxLayout()
        cr.addWidget(QLabel("Custom (m):"))
        self._dist_edit = QLineEdit("1.0"); self._dist_edit.setFixedWidth(80)
        cr.addWidget(self._dist_edit)
        cr.addWidget(btn("FWD","#0e1820","#1a4060","#2a80c0",h=32,w=60).on_click(lambda: self._send_custom(1)))
        cr.addWidget(btn("REV","#1a0808","#402010","#804020",h=32,w=60).on_click(lambda: self._send_custom(-1)))
        cr.addStretch()
        dv.addLayout(cr)
        root.addWidget(dg)

        # ── TURN CONTROL ──────────────────────────────────────────────────────
        trn = QGroupBox("TURN CONTROL  —  encoder-based pivot and arc turns")
        tv2 = QVBoxLayout(trn); tv2.setSpacing(7)

        # Turn speed slider
        spd_row = QHBoxLayout()
        spd_row.addWidget(QLabel("Turn speed  PWM:"))
        self._turn_spd = QSlider(Qt.Horizontal)
        self._turn_spd.setRange(20, 190); self._turn_spd.setValue(100)
        self._turn_spd.setFixedHeight(24)
        self._turn_spd_lbl = QLabel("100")
        self._turn_spd_lbl.setFixedWidth(36)
        self._turn_spd_lbl.setStyleSheet("color:#50a0c8;font-weight:bold;font-size:14px;")
        self._turn_spd.valueChanged.connect(lambda v: self._turn_spd_lbl.setText(str(v)))
        spd_row.addWidget(self._turn_spd, 1); spd_row.addWidget(self._turn_spd_lbl)
        tv2.addLayout(spd_row)

        # Pivot presets
        tv2.addWidget(QLabel("Pivot turn presets  (positive = CCW, negative = CW):")
                      .__class__(  # just a QLabel
                          "Pivot presets  (+= CCW,  −= CW):"))
        p_row = QHBoxLayout()
        pivot_presets = [
            ("↺ 90° CCW",  90), ("↺ 45° CCW",  45), ("↺ 30° CCW",  30),
            ("↻ 30° CW",  -30), ("↻ 45° CW",  -45), ("↻ 90° CW",  -90),
            ("↻ 180° CW",-180),
        ]
        for lbl, deg in pivot_presets:
            p_row.addWidget(
                btn(lbl,"#0e1820","#1a3858","#2a5878",h=34)
                .on_click(lambda _, d=deg: self._send_pivot(d)))
        p_row.addStretch()
        tv2.addLayout(p_row)

        # Custom pivot
        cpt = QHBoxLayout()
        cpt.addWidget(QLabel("Custom pivot:"))
        self._pt_deg = QDoubleSpinBox()
        self._pt_deg.setRange(-720, 720); self._pt_deg.setValue(90); self._pt_deg.setDecimals(1)
        self._pt_deg.setFixedWidth(90)
        cpt.addWidget(self._pt_deg)
        cpt.addWidget(QLabel("degrees"))
        cpt.addWidget(
            btn("SEND PIVOT","#0e1820","#2a4060","#4080c0",h=34,w=140)
            .on_click(lambda: self._send_pivot(self._pt_deg.value())))
        cpt.addStretch()
        tv2.addLayout(cpt)

        # Arc turn: independent per-wheel distance
        tv2.addWidget(self._hsep())
        tv2.addWidget(QLabel("Arc turn  (independent per-wheel distance, + = forward):"))

        arc_row = QHBoxLayout()
        arc_row.addWidget(QLabel("Left wheel (mm):"))
        self._arc_bl = QDoubleSpinBox()
        self._arc_bl.setRange(-9999, 9999); self._arc_bl.setValue(500); self._arc_bl.setDecimals(0)
        self._arc_bl.setFixedWidth(90)
        arc_row.addWidget(self._arc_bl)
        arc_row.addWidget(QLabel("   Right wheel (mm):"))
        self._arc_br = QDoubleSpinBox()
        self._arc_br.setRange(-9999, 9999); self._arc_br.setValue(300); self._arc_br.setDecimals(0)
        self._arc_br.setFixedWidth(90)
        arc_row.addWidget(self._arc_br)
        arc_row.addWidget(
            btn("SEND ARC","#0e1820","#2a4060","#4080c0",h=34,w=120)
            .on_click(self._send_arc))
        arc_row.addStretch()
        tv2.addLayout(arc_row)

        # Arc presets
        arc_presets_row = QHBoxLayout()
        arc_presets_row.addWidget(QLabel("Presets:"))
        arc_presets = [
            ("Gentle right\nL500 R300",  500,  300),
            ("Sharp right\nL500 R0",     500,    0),
            ("Gentle left\nL300 R500",   300,  500),
            ("Sharp left\nL0 R500",        0,  500),
            ("Spin right\nL500 R-500",   500, -500),
            ("Spin left\nL-500 R500",   -500,  500),
        ]
        for lbl, bl, br in arc_presets:
            arc_presets_row.addWidget(
                btn(lbl,"#0a1820","#183050","#286080",h=46)
                .on_click(lambda _,a=bl,b=br: self._preset_arc(a,b)))
        arc_presets_row.addStretch()
        tv2.addLayout(arc_presets_row)

        root.addWidget(trn)

        # ── ACTUATOR ──────────────────────────────────────────────────────────
        ag = QGroupBox("ACTUATOR")
        av = QHBoxLayout(ag)
        for lbl, cmd in [("DIG",0xA7),("DRIVE POS",0xA9),("DUMP",0xB3),("CALIBRATE",0xCA)]:
            av.addWidget(
                btn(lbl,"#1a1028","#3a1060","#7030c0",h=36,w=110)
                .on_click(lambda _,c=cmd: self._send_raw_cmd(c,0,0,0)))
        av.addStretch()
        root.addWidget(ag)

        # ── STACK / RVIZ ──────────────────────────────────────────────────────
        sg = QGroupBox("STACK LAUNCH  —  SSH to miniPC")
        sv = QHBoxLayout(sg)
        sv.addWidget(btn("Autonomous stack","#0e1820","#1a4060","#2a80c0",h=36).on_click(self._launch_auto))
        sv.addWidget(btn("Full miniPC stack","#0e1820","#1a4060","#2a80c0",h=36).on_click(self._launch_full))
        sv.addWidget(btn("Open RViz2 (local)","#0e1018","#1a3040","#2a5060",h=36).on_click(self._launch_rviz))
        sv.addStretch()
        root.addWidget(sg)
        root.addStretch()
        return w

    def _hsep(self):
        f = QFrame(); f.setFrameShape(QFrame.HLine)
        f.setStyleSheet("color:#1a2a3a;"); return f

    # ═════════════════════════════════════════════════════════════════════════
    # MISSION TAB
    # ═════════════════════════════════════════════════════════════════════════

    def _tab_mission(self):
        w = QWidget(); root = QVBoxLayout(w)
        root.setSpacing(6); root.setContentsMargins(10,10,10,10)

        # File row
        ql = QHBoxLayout(); ql.addWidget(QLabel("YAML:"))
        self._yaml_combo = QComboBox(); self._yaml_combo.setMinimumWidth(200)
        ql.addWidget(self._yaml_combo, 1)
        for lbl, fn in [("Load",self._quick_load),("Refresh",self._refresh_yamls),("Browse",self._browse_yaml)]:
            ql.addWidget(btn(lbl,"#0e1820","#1a4060","#2a80c0",h=28,w=80).on_click(fn))
        root.addLayout(ql)
        self._file_lbl = QLabel("No file loaded")
        self._file_lbl.setStyleSheet("color:#405868;font-size:12px;")
        root.addWidget(self._file_lbl)

        # Warning note
        note = QLabel(
            "⚠  distance_m in YAML must be in METRES — e.g. 1.5, not 1500.\n"
            "   timeout_s should be generous (60–120 s).  "
            "The sequencer logs step-by-step timing on the miniPC.")
        note.setStyleSheet(
            "color:#906030;font-size:11px;background:#1a1208;"
            "border:1px solid #604020;border-radius:4px;padding:4px 8px;")
        note.setWordWrap(True)
        root.addWidget(note)

        spl = QSplitter(Qt.Horizontal)
        spl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Left: step list
        lw = QWidget(); lv = QVBoxLayout(lw); lv.setContentsMargins(0,0,4,0); lv.setSpacing(4)
        lv.addWidget(QLabel("Mission steps:"))
        self._scroll = QScrollArea(); self._scroll.setWidgetResizable(True)
        self._scroll.setStyleSheet("QScrollArea{background:#080b10;border:1px solid #182030;}")
        self._step_box = QWidget(); self._step_lay = QVBoxLayout(self._step_box)
        self._step_lay.setSpacing(3); self._step_lay.setAlignment(Qt.AlignTop)
        self._scroll.setWidget(self._step_box)
        lv.addWidget(self._scroll, 1)
        lr = QHBoxLayout()
        lr.addWidget(btn("Clear","#0e1018","#1a2030","#2a3848",h=28).on_click(self._clear_steps))
        lr.addWidget(btn("Save YAML","#0e1018","#1c3040","#2a5060",h=28).on_click(self._save_yaml))
        lv.addLayout(lr)
        spl.addWidget(lw)

        # Right: add-step panel
        rw = QWidget(); rv = QVBoxLayout(rw); rv.setContentsMargins(4,0,0,0); rv.setSpacing(6)
        rv.addWidget(QLabel("Add step:"))
        self._act_combo = QComboBox(); self._act_combo.addItems(list(ACTION_DEFAULTS.keys()))
        self._act_combo.currentTextChanged.connect(self._refresh_param_editor)
        rv.addWidget(self._act_combo)
        self._param_area = QWidget(); self._param_lay = QVBoxLayout(self._param_area)
        self._param_lay.setSpacing(4); rv.addWidget(self._param_area)
        self._param_wids = {}; self._refresh_param_editor(self._act_combo.currentText())
        rv.addWidget(btn("+ Add Step","#0e1820","#1a4060","#2a80c0",h=36).on_click(self._add_step))
        rv.addStretch()
        spl.addWidget(rw)
        spl.setSizes([420, 300])
        root.addWidget(spl, 1)

        nr = QHBoxLayout(); nr.addWidget(QLabel("Name:"))
        self._name_edit = QLineEdit("Untitled mission"); nr.addWidget(self._name_edit, 1)
        root.addLayout(nr)

        # ── Launch panel ──────────────────────────────────────────────────────
        lg = QGroupBox("RUN MISSION")
        lv2 = QVBoxLayout(lg); lv2.setSpacing(6)

        # Row 1: sequencer status + start sequencer on miniPC
        sq_row = QHBoxLayout()
        self._seq_led = LED("off")
        self._seq_lbl = QLabel("Sequencer: unknown")
        self._seq_lbl.setStyleSheet("color:#506070;font-size:12px;")
        sq_row.addWidget(self._seq_led); sq_row.addWidget(self._seq_lbl, 1)
        sq_row.addWidget(
            btn("Start Sequencer on miniPC","#0e1820","#1a4060","#2a80c0",h=34,w=240)
            .on_click(self._start_sequencer))
        sq_row.addWidget(
            btn("Check status","#0e1018","#1a3040","#2a5060",h=34,w=120)
            .on_click(self._check_sequencer))
        lv2.addLayout(sq_row)

        lv2.addWidget(QLabel(
            "The sequencer must be running on the miniPC before you can start a mission.\n"
            "Click 'Start Sequencer' above, wait for the LED to go green, then START MISSION."))

        # Row 2: main start/abort + progress
        run_row = QHBoxLayout()
        self._run_btn   = btn("▶  START MISSION","#081a08","#1a5020","#28a040",h=44,w=210)
        self._abort_btn = btn("■  ABORT",         "#1a0808","#501010","#a02020",h=44,w=110)
        self._run_btn.on_click(self._start_mission)
        self._abort_btn.on_click(self._abort_mission)
        run_row.addWidget(self._run_btn); run_row.addWidget(self._abort_btn)
        run_row.addStretch()
        lv2.addLayout(run_row)

        # Status label + step progress
        self._mstatus = QLabel("Idle — load or build a mission, start the sequencer, then press START")
        self._mstatus.setStyleSheet("color:#506878;font-size:12px;")
        self._mstatus.setWordWrap(True)
        lv2.addWidget(self._mstatus)

        prog_row = QHBoxLayout()
        self._mprog = QProgressBar()
        self._mprog.setFixedHeight(10); self._mprog.setTextVisible(False)
        self._mprog.setStyleSheet(
            "QProgressBar{background:#0e1218;border:none;border-radius:5px;}"
            "QProgressBar::chunk{background:#28a040;border-radius:5px;}")
        prog_row.addWidget(self._mprog, 1)
        self._mprog_lbl = QLabel("0 / 0")
        self._mprog_lbl.setStyleSheet("color:#507868;font-size:12px;min-width:48px;")
        prog_row.addWidget(self._mprog_lbl)
        lv2.addLayout(prog_row)

        root.addWidget(lg)

        self._refresh_yamls()
        return w

    # ═════════════════════════════════════════════════════════════════════════
    # LOG TAB
    # ═════════════════════════════════════════════════════════════════════════

    def _tab_log(self):
        w = QWidget(); root = QVBoxLayout(w); root.setContentsMargins(10,10,10,10)
        self.log_view = QTextEdit(); self.log_view.setReadOnly(True)
        root.addWidget(self.log_view, 1)
        root.addWidget(btn("Clear log","#0e1018","#1a2030","#2a3040",h=28,w=90)
                       .on_click(self.log_view.clear))
        return w

    # ═════════════════════════════════════════════════════════════════════════
    # PARAM EDITOR  (mission tab right panel)
    # ═════════════════════════════════════════════════════════════════════════

    def _refresh_param_editor(self, action):
        while self._param_lay.count():
            item = self._param_lay.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        self._param_wids.clear()
        for key, default in ACTION_DEFAULTS.get(action, {}).items():
            row = QHBoxLayout()
            lbl = QLabel(f"{key}:")
            lbl.setStyleSheet("color:#6080a0;font-size:12px;min-width:120px;")
            row.addWidget(lbl)
            if key == "target":
                cb = QComboBox(); cb.addItems(["dig","drive","dump"])
                cb.setCurrentText(str(default)); cb.setFixedHeight(28)
                row.addWidget(cb); self._param_wids[key] = cb
            elif isinstance(default, bool):
                ck = QCheckBox(); ck.setChecked(default)
                row.addWidget(ck); self._param_wids[key] = ck
            elif isinstance(default, float):
                sb = QDoubleSpinBox(); sb.setDecimals(3); sb.setRange(-9999, 9999)
                sb.setValue(default); sb.setFixedHeight(28)
                row.addWidget(sb); self._param_wids[key] = sb
            elif isinstance(default, int):
                sb = QSpinBox(); sb.setRange(0, 9999); sb.setValue(default)
                sb.setFixedHeight(28); row.addWidget(sb); self._param_wids[key] = sb
            else:
                le = QLineEdit(str(default)); le.setFixedHeight(28)
                row.addWidget(le); self._param_wids[key] = le
            self._param_lay.addLayout(row)

    def _read_params(self):
        action = self._act_combo.currentText()
        params = {}
        for key, default in ACTION_DEFAULTS.get(action, {}).items():
            w = self._param_wids.get(key)
            if w is None: continue
            if isinstance(w, QCheckBox):                    params[key] = w.isChecked()
            elif isinstance(w, (QDoubleSpinBox, QSpinBox)): params[key] = w.value()
            elif isinstance(w, QComboBox):                  params[key] = w.currentText()
            else:
                raw = w.text().strip()
                if isinstance(default, float):
                    try: params[key] = float(raw)
                    except: params[key] = default
                elif isinstance(default, int):
                    try: params[key] = int(raw)
                    except: params[key] = default
                else: params[key] = raw
        return params

    # ═════════════════════════════════════════════════════════════════════════
    # STEP MANAGEMENT
    # ═════════════════════════════════════════════════════════════════════════

    def _add_step(self):
        self._insert(self._act_combo.currentText(), self._read_params())

    def _insert(self, action, params):
        sw = StepWidget(action, params)
        sw.sig_remove.connect(self._rm); sw.sig_up.connect(self._mu); sw.sig_down.connect(self._md)
        self._steps.append(sw); self._step_lay.addWidget(sw)
        self._mstatus.setText(f"{len(self._steps)} step(s)")

    def _rm(self, sw):
        if sw in self._steps:
            self._steps.remove(sw); self._step_lay.removeWidget(sw); sw.deleteLater()
            self._mstatus.setText(f"{len(self._steps)} step(s)")

    def _mu(self, sw):
        i = self._steps.index(sw)
        if i > 0: self._steps[i], self._steps[i-1] = self._steps[i-1], self._steps[i]; self._rebuild()

    def _md(self, sw):
        i = self._steps.index(sw)
        if i < len(self._steps)-1: self._steps[i], self._steps[i+1] = self._steps[i+1], self._steps[i]; self._rebuild()

    def _rebuild(self):
        for sw in self._steps: self._step_lay.removeWidget(sw)
        for sw in self._steps: self._step_lay.addWidget(sw)

    def _clear_steps(self):
        for sw in list(self._steps): self._rm(sw)
        self._yaml_path = ""; self._file_lbl.setText("No file loaded")

    def _mission_dict(self):
        return {"mission": {"name": self._name_edit.text(),
                            "steps": [sw.to_dict() for sw in self._steps]}}

    # ═════════════════════════════════════════════════════════════════════════
    # YAML I/O
    # ═════════════════════════════════════════════════════════════════════════

    def _refresh_yamls(self):
        self._yaml_combo.clear()
        ws = os.path.expanduser("~/lunar_rover_ws")
        files = sorted(glob.glob(f"{ws}/*.yaml") + glob.glob(f"{ws}/*.yml"))
        for f in files: self._yaml_combo.addItem(Path(f).name, f)
        if not files: self._yaml_combo.addItem("(no YAML files found)", "")

    def _quick_load(self):
        p = self._yaml_combo.currentData()
        if p and os.path.exists(p): self._load(p)
        else: self._log("No valid YAML selected in dropdown")

    def _browse_yaml(self):
        p, _ = QFileDialog.getOpenFileName(self, "Load YAML",
                                           os.path.expanduser("~/lunar_rover_ws"),
                                           "YAML (*.yaml *.yml)")
        if p: self._load(p)

    def _load(self, path):
        if not YAML_AVAILABLE:
            self._log("pyyaml missing: pip3 install pyyaml --break-system-packages"); return
        try:
            with open(path) as f: doc = yaml.safe_load(f)
            mission = doc.get("mission", doc)
            steps = mission.get("steps", []); name = mission.get("name", Path(path).stem)
            self._clear_steps(); self._name_edit.setText(name)
            for s in steps: self._insert(s.get("action","stop"), s.get("params",{}))
            self._yaml_path = path
            self._file_lbl.setText(f"Loaded: {Path(path).name}  ({len(steps)} steps)")
            self._log(f"Loaded {len(steps)} steps from {Path(path).name}")
        except Exception as e:
            self._log(f"YAML load error: {e}")

    def _save_yaml(self):
        if not YAML_AVAILABLE:
            self._log("pyyaml missing"); return
        default = self._yaml_path or os.path.expanduser("~/lunar_rover_ws/mission.yaml")
        path, _ = QFileDialog.getSaveFileName(self, "Save YAML", default, "YAML (*.yaml *.yml)")
        if not path: return
        try:
            with open(path, "w") as f:
                yaml.dump(self._mission_dict(), f, default_flow_style=False, sort_keys=False)
            self._yaml_path = path
            self._file_lbl.setText(f"Saved: {Path(path).name}")
            self._log(f"Saved to {path}"); self._refresh_yamls()
        except Exception as e:
            self._log(f"Save error: {e}")

    # ═════════════════════════════════════════════════════════════════════════
    # MISSION RUN
    # ═════════════════════════════════════════════════════════════════════════

    def _ros2_pub_cmd(self, topic, msg_type, value_str, timeout=12):
        """
        Publish a single message via `ros2 topic pub --once` in a subprocess.
        Returns (success_bool, output_str).
        """
        ws_setup = os.path.expanduser("~/lunar_rover_ws/install/setup.bash")
        ros_env = (
            "source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null; "
            + (f"source {ws_setup} 2>/dev/null; " if os.path.exists(ws_setup) else "")
            + "export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 "
            "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; "
            f"ros2 topic pub --once {topic} {msg_type} \"{value_str}\""
        )
        try:
            r = subprocess.run(["bash", "-c", ros_env],
                               capture_output=True, text=True, timeout=timeout)
            return r.returncode == 0, r.stdout + r.stderr
        except subprocess.TimeoutExpired:
            return False, "timeout"
        except Exception as e:
            return False, str(e)

    def _ensure_pubs(self):
        """Lazy-init ROS publishers used by the quick-command buttons."""
        if not ROS_AVAILABLE or self._start_pub is not None: return
        try:
            if not rclpy.ok(): rclpy.init()
            self._ros_pub_nd = rclpy.create_node("rover_gui_pub")
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
            self._start_pub = self._ros_pub_nd.create_publisher(Bool,             "/mission/start",         q)
            self._file_pub  = self._ros_pub_nd.create_publisher(String,           "/mission/file",           q)
            self._dist_pub  = self._ros_pub_nd.create_publisher(Float32,          "/nav/arduino_dist_cmd",   q)
            self._cmd_pub   = self._ros_pub_nd.create_publisher(Float32MultiArray, "/nav/arduino_cmd",       q)
            self._turn_pub  = self._ros_pub_nd.create_publisher(Float32MultiArray, "/nav/arduino_turn_cmd",  q)
            from rclpy.executors import SingleThreadedExecutor
            ex = SingleThreadedExecutor(); ex.add_node(self._ros_pub_nd)
            for _ in range(5): ex.spin_once(timeout_sec=0.02)
            self._log("ROS publishers ready")
        except Exception as e:
            self._log(f"ROS publisher init: {e}")

    def _start_mission(self):
        if not self._steps:
            self._log("No steps — add steps or load a YAML first"); return
        if not YAML_AVAILABLE:
            self._log("pyyaml missing — cannot auto-save"); return

        # Save YAML locally on the laptop
        local_path = os.path.expanduser("~/lunar_rover_ws/_gui_mission.yaml")
        try:
            os.makedirs(os.path.dirname(local_path), exist_ok=True)
            with open(local_path, "w") as f:
                yaml.dump(self._mission_dict(), f, default_flow_style=False, sort_keys=False)
            self._log(f"Auto-saved locally: {local_path}")
        except Exception as e:
            self._log(f"Auto-save failed: {e}"); return

        def _send():
            # 1. SCP the YAML file to the miniPC
            remote_path = f"{MINIPC_WS}/_gui_mission.yaml"   # tilde, expanded by miniPC shell
            self._log("Copying YAML to miniPC via SCP...")
            try:
                scp = subprocess.run(
                    ["scp", "-o", "ConnectTimeout=8", "-o", "StrictHostKeyChecking=no",
                     local_path, f"{MINIPC_USER}@{MINIPC_IP}:{remote_path}"],
                    capture_output=True, text=True, timeout=15)
            except Exception as e:
                self._log(f"SCP error: {e}"); return
            if scp.returncode != 0:
                self._log(f"SCP failed: {scp.stderr or scp.stdout}")
                self._log("Is SSH key auth configured? Run: ssh-copy-id cheese@192.168.0.102"); return
            self._log("YAML copied to miniPC OK")

            # 2. Send /mission/file (miniPC-side path, tilde is fine — sequencer calls expanduser)
            self._log("Sending /mission/file ...")
            ok, out = self._ros2_pub_cmd(
                "/mission/file", "std_msgs/msg/String",
                f"data: '{remote_path}'")   # single quotes so shell doesn't eat the tilde
            self._log(f"  /mission/file: {'OK' if ok else 'FAILED'}")
            if not ok:
                self._log(f"  output: {out[:250]}")
                self._log("  Is nav_mission_sequencer.py running on the miniPC?")
                self._log("  Start it: ssh cheese@192.168.0.102 "
                          "'cd ~/lunar_rover_ws && python3 nav_mission_sequencer.py &'")
                return

            time.sleep(0.6)

            # 3. Send /mission/start True
            self._log("Sending /mission/start True ...")
            ok2, out2 = self._ros2_pub_cmd(
                "/mission/start", "std_msgs/msg/Bool", "data: true")
            if ok2:
                self._log("  Mission started! Watch the miniPC terminal for step logs.")
            else:
                self._log(f"  /mission/start FAILED: {out2[:250]}")

        threading.Thread(target=_send, daemon=True).start()
        self._mstatus.setText("Mission starting..."); self._mprog.setValue(5)

    def _abort_mission(self):
        def _send():
            ok, _ = self._ros2_pub_cmd("/mission/start", "std_msgs/msg/Bool", "data: false")
            self._log(f"Abort {'sent' if ok else 'failed'}")
        threading.Thread(target=_send, daemon=True).start()
        self._mstatus.setText("Aborted"); self._mprog.setValue(0)

    # ═════════════════════════════════════════════════════════════════════════
    # QUICK COMMANDS
    # ═════════════════════════════════════════════════════════════════════════

    def _send_dist(self, m):
        self._ensure_pubs()
        if ROS_AVAILABLE and self._dist_pub:
            msg = Float32(); msg.data = float(m); self._dist_pub.publish(msg)
            self._log(f"Drive {m:+.1f} m")
        else: self._log("ROS unavailable — is nav_arduino_bridge.py running on miniPC?")

    def _send_custom(self, sign):
        try: self._send_dist(float(self._dist_edit.text()) * sign)
        except ValueError: self._log("Invalid distance value")

    def _send_raw_cmd(self, device, speed, direction, lobyte):
        self._ensure_pubs()
        if ROS_AVAILABLE and self._cmd_pub:
            m = Float32MultiArray()
            m.data = [float(device), float(speed), float(direction), float(lobyte)]
            self._cmd_pub.publish(m)
            self._log(f"Cmd 0x{device:02X}  sp={speed}  dir={direction}")
        else: self._log("ROS unavailable")

    def _send_pivot(self, degrees: float):
        """Send a pivot turn via /nav/arduino_turn_cmd [arc_mm, speed, clockwise_int]."""
        speed  = self._turn_spd.value()
        arc_mm = 350.0 * abs(math.radians(degrees))   # (700/2) * |rad|
        cw     = 1 if degrees < 0 else 0              # negative = CW
        self._ensure_pubs()
        if ROS_AVAILABLE and self._turn_pub:
            m = Float32MultiArray(); m.data = [arc_mm, float(speed), float(cw)]
            self._turn_pub.publish(m)
            self._log(f"Pivot {degrees:+.1f}°  arc={arc_mm:.0f} mm  "
                      f"{'CW' if cw else 'CCW'}  speed={speed}")
        else: self._log("ROS unavailable — is nav_arduino_bridge.py running on miniPC?")

    def _send_arc(self):
        """Send a custom arc turn (independent per-wheel)."""
        bl_mm = self._arc_bl.value()
        br_mm = self._arc_br.value()
        self._send_arc_mm(bl_mm, br_mm)

    def _preset_arc(self, bl_mm: float, br_mm: float):
        self._arc_bl.setValue(bl_mm); self._arc_br.setValue(br_mm)
        self._send_arc_mm(bl_mm, br_mm)

    def _send_arc_mm(self, bl_mm: float, br_mm: float):
        speed = self._turn_spd.value()
        self._ensure_pubs()
        if not ROS_AVAILABLE or self._cmd_pub is None:
            self._log("ROS unavailable"); return

        def _enc(mm):
            u = int(min(0x7FFF, max(0, round(abs(mm)))))
            c = ((1 if mm < 0 else 0) << 15) | u
            return (c >> 8) & 0xFF, c & 0xFF

        bl_db, bl_lo = _enc(bl_mm)
        br_db, br_lo = _enc(br_mm)

        def _seq():
            m = Float32MultiArray()
            m.data = [0xC8, float(speed), float(bl_db), float(bl_lo)]
            self._cmd_pub.publish(m); time.sleep(0.05)
            m.data = [0xC9, float(speed), float(br_db), float(br_lo)]
            self._cmd_pub.publish(m); time.sleep(0.05)
            m.data = [0xE8, 0.0, 0.0, 0.0]
            self._cmd_pub.publish(m)
            self._log(f"Arc  L={bl_mm:+.0f} mm  R={br_mm:+.0f} mm  speed={speed}")
        threading.Thread(target=_seq, daemon=True).start()

    # ═════════════════════════════════════════════════════════════════════════
    # TELEOP
    # ═════════════════════════════════════════════════════════════════════════

    def _start_teleop(self):
        self._log("SSH → miniPC: launching joy_node + joy_to_arduino...")
        self._joy_led.set_color("yellow"); self._joy_lbl.setText("starting...")

        # Original working pattern:
        #   1. Kill old instances
        #   2. Launch joy_node, wait 3 s for it to initialise
        #   3. Launch joy_to_arduino
        #   4. disown -a  → SSH exits immediately, background jobs keep running
        #   5. echo LAUNCH_DONE  → sentinel we check for
        remote_script = (
            'source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null\n'
            f'[ -f {MINIPC_WS}/install/setup.bash ] && '
            f'source {MINIPC_WS}/install/setup.bash\n'
            'export ROS_DOMAIN_ID=42\n'
            'export ROS_LOCALHOST_ONLY=0\n'
            'export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET\n'
            'pkill -f joy_to_arduino 2>/dev/null\n'
            'pkill -f joy_node        2>/dev/null\n'
            'sleep 1\n'
            'ros2 run joy joy_node > /tmp/joy_node.log 2>&1 &\n'
            'sleep 3\n'
            f'python3 {MINIPC_WS}/joy_to_arduino.py > /tmp/joy_arduino.log 2>&1 &\n'
            'sleep 1\n'
            'disown -a\n'
            'echo LAUNCH_DONE\n'
        )

        def run():
            try:
                proc = subprocess.Popen(
                    ["ssh", "-o", "ConnectTimeout=8", "-o", "StrictHostKeyChecking=no",
                     f"{MINIPC_USER}@{MINIPC_IP}", "bash", "-s"],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True)
                stdout, _ = proc.communicate(input=remote_script, timeout=20)
                self._log(f"  SSH out: {stdout.strip()[:200]}")
                if "LAUNCH_DONE" in stdout:
                    self._joy_led.set_color("green")
                    self._joy_lbl.setText("running on miniPC")
                    self._log("Teleop ready. Controller should respond in ~3 s.")
                    self._log("  joy_node log:    ssh cheese@miniPC cat /tmp/joy_node.log")
                    self._log("  joy_arduino log: ssh cheese@miniPC cat /tmp/joy_arduino.log")
                else:
                    self._joy_led.set_color("red"); self._joy_lbl.setText("failed — see LOG")
                    self._log("Teleop failed. Check:")
                    self._log("  1. ssh cheese@192.168.0.102 echo ok")
                    self._log("  2. ros2 pkg list | grep joy   (on miniPC)")
                    self._log(f"  full output: {stdout[:400]}")
            except subprocess.TimeoutExpired:
                self._joy_led.set_color("red"); self._joy_lbl.setText("SSH timeout")
                self._log("SSH timed out after 20 s — check miniPC reachable")
            except Exception as e:
                self._joy_led.set_color("red"); self._joy_lbl.setText("error")
                self._log(f"SSH error: {e}")

        threading.Thread(target=run, daemon=True).start()

    def _stop_teleop(self):
        remote_script = (
            'pkill -f joy_to_arduino 2>/dev/null\n'
            'pkill -f joy_node        2>/dev/null\n'
            'echo STOPPED\n'
        )
        def run():
            try:
                proc = subprocess.Popen(
                    ["ssh", "-o", "ConnectTimeout=8", "-o", "StrictHostKeyChecking=no",
                     f"{MINIPC_USER}@{MINIPC_IP}", "bash", "-s"],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True)
                proc.communicate(input=remote_script, timeout=12)
                self._joy_led.set_color("off"); self._joy_lbl.setText("stopped")
                self._log("Teleop stopped")
            except Exception as e:
                self._log(f"Stop teleop error: {e}")
        threading.Thread(target=run, daemon=True).start()

    # ═════════════════════════════════════════════════════════════════════════
    # STACK / RVIZ
    # ═════════════════════════════════════════════════════════════════════════

    def _ssh_bg(self, cmd, label):
        env = ("source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null; "
               f"source {MINIPC_WS}/install/setup.bash 2>/dev/null; "
               "export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; ")
        def run():
            try:
                r = subprocess.run(
                    ["ssh","-o","ConnectTimeout=10","-o","StrictHostKeyChecking=no",
                     f"{MINIPC_USER}@{MINIPC_IP}","bash","-c", env+cmd],
                    capture_output=True, text=True, timeout=20)
                self._log(f"[{label}] {'OK' if r.returncode==0 else 'FAILED'}: "
                          + (r.stderr or r.stdout or "")[:200])
            except Exception as e:
                self._log(f"[{label}] {e}")
        threading.Thread(target=run, daemon=True).start()

    def _launch_auto(self):
        self._log("Starting autonomous stack on miniPC...")
        self._ssh_bg(f"nohup bash {MINIPC_WS}/full_launch_autonomous.sh > /tmp/minipc_auto.log 2>&1 &", "auto")

    def _launch_full(self):
        self._log("Starting full miniPC stack...")
        self._ssh_bg(f"nohup bash {MINIPC_WS}/full_launch_minipc.sh > /tmp/minipc_launch.log 2>&1 &", "full")

    def _launch_rviz(self):
        cmd = ["rviz2", "-d", RVIZ_CONFIG] if os.path.exists(RVIZ_CONFIG) else ["rviz2"]
        try:
            subprocess.Popen(cmd); self._log("RViz2 launched locally")
        except FileNotFoundError:
            self._log("rviz2 not found — source ROS2 on the laptop first")

    # ═════════════════════════════════════════════════════════════════════════
    # ROS MONITOR + CONN CHECK + LOG
    # ═════════════════════════════════════════════════════════════════════════

    def _start_sequencer(self):
        """SSH to miniPC and start nav_mission_sequencer.py in the background."""
        self._log("Starting sequencer on miniPC...")
        self._seq_led.set_color("yellow"); self._seq_lbl.setText("Sequencer: starting...")

        remote_script = (
            'source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null\n'
            f'[ -f {MINIPC_WS}/install/setup.bash ] && '
            f'source {MINIPC_WS}/install/setup.bash\n'
            'export ROS_DOMAIN_ID=42\n'
            'export ROS_LOCALHOST_ONLY=0\n'
            'export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET\n'
            'pkill -f nav_mission_sequencer 2>/dev/null\n'
            'sleep 1\n'
            f'python3 {MINIPC_WS}/nav_mission_sequencer.py > /tmp/sequencer.log 2>&1 &\n'
            'sleep 3\n'
            'disown -a\n'
            'echo SEQ_STARTED\n'
        )
        def run():
            try:
                proc = subprocess.Popen(
                    ["ssh", "-o", "ConnectTimeout=8", "-o", "StrictHostKeyChecking=no",
                     f"{MINIPC_USER}@{MINIPC_IP}", "bash", "-s"],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True)
                stdout, _ = proc.communicate(input=remote_script, timeout=20)
                if "SEQ_STARTED" in stdout:
                    self._seq_led.set_color("green")
                    self._seq_lbl.setText("Sequencer: running on miniPC")
                    self._log("Sequencer started OK")
                    self._log("  Log: ssh cheese@miniPC cat /tmp/sequencer.log")
                else:
                    self._seq_led.set_color("red")
                    self._seq_lbl.setText("Sequencer: start FAILED")
                    self._log(f"Sequencer start failed: {stdout[:300]}")
            except subprocess.TimeoutExpired:
                self._seq_led.set_color("red"); self._seq_lbl.setText("Sequencer: SSH timeout")
                self._log("SSH timed out starting sequencer")
            except Exception as e:
                self._seq_led.set_color("red"); self._log(f"Error: {e}")
        threading.Thread(target=run, daemon=True).start()

    def _check_sequencer(self):
        """Check whether nav_mission_sequencer is running on the miniPC."""
        self._log("Checking sequencer status on miniPC...")
        def run():
            try:
                r = subprocess.run(
                    ["ssh", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=no",
                     f"{MINIPC_USER}@{MINIPC_IP}", "bash", "-c",
                     "pgrep -af nav_mission_sequencer && echo RUNNING || echo NOT_RUNNING"],
                    capture_output=True, text=True, timeout=10)
                out = r.stdout.strip()
                if "RUNNING" in out and "NOT_RUNNING" not in out:
                    self._seq_led.set_color("green")
                    self._seq_lbl.setText("Sequencer: running")
                    self._log(f"Sequencer is running: {out.splitlines()[0][:80]}")
                else:
                    self._seq_led.set_color("red")
                    self._seq_lbl.setText("Sequencer: NOT running")
                    self._log("Sequencer is NOT running — click 'Start Sequencer on miniPC'")
            except Exception as e:
                self._log(f"Check error: {e}")
        threading.Thread(target=run, daemon=True).start()

    def _start_ros_monitor(self):
        self._ros_mon = RosMonitor()
        self._ros_mon.mission_status.connect(self._on_mission_status)
        self._ros_mon.start()

    @pyqtSlot(dict)
    def _on_mission_status(self, d):
        step  = d.get("step", 0); total = d.get("total", 0)
        name  = d.get("step_name", ""); run = d.get("running", False)
        if total > 0:
            self._mprog.setValue(int(step / total * 100))
            self._mprog_lbl.setText(f"{step+1 if run else step} / {total}")
        else:
            self._mprog.setValue(0); self._mprog_lbl.setText("0 / 0")
        if run:
            self._mstatus.setText(f"Running  step {step+1}/{total}: {name}")
            self._mstatus.setStyleSheet("color:#40a860;font-size:12px;")
            self._seq_led.set_color("green"); self._seq_lbl.setText("Sequencer: running mission")
        elif step > 0 and total > 0:
            self._mstatus.setText(f"Complete  ({total} steps done)")
            self._mstatus.setStyleSheet("color:#50a870;font-size:12px;")
            self._seq_led.set_color("green"); self._seq_lbl.setText("Sequencer: idle (last mission complete)")
        else:
            self._mstatus.setText("Idle — load or build a mission, start the sequencer, then press START")
            self._mstatus.setStyleSheet("color:#506878;font-size:12px;")

    def _start_conn_check(self):
        self._ct = QTimer(); self._ct.timeout.connect(self._check_conn)
        self._ct.start(8000); self._check_conn()

    def _check_conn(self):
        def run():
            r = subprocess.run(f"ping -c1 -W2 {MINIPC_IP}", shell=True, capture_output=True)
            ok = r.returncode == 0
            self._conn_led.set_color("green" if ok else "red")
            self._conn_lbl.setText(f"miniPC {MINIPC_IP}: {'online' if ok else 'offline'}")
        threading.Thread(target=run, daemon=True).start()

    def _log(self, msg):
        self._log_sig.emit(str(msg))

    def _log_direct(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.append(f"<span style='color:#304858'>[{ts}]</span> {msg}")

    def closeEvent(self, e):
        if hasattr(self, "_ros_mon"): self._ros_mon.stop()
        e.accept()


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("Courier New", 13))
    w = GUI(); w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()