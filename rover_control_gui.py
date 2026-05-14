#!/usr/bin/env python3
"""
rover_mission_gui.py  —  Lunar Rover Mission Control
Run on the LAPTOP:  python3 ~/lunar_rover_ws/rover_mission_gui.py
"""

import os, sys, glob, json, subprocess, threading, time
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit, QSizePolicy, QFrame,
    QLineEdit, QTabWidget, QFileDialog, QScrollArea, QComboBox,
    QDoubleSpinBox, QSpinBox, QCheckBox, QProgressBar, QSplitter,
    QSlider,
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

MINIPC_USER = "cheese"
MINIPC_IP   = "192.168.0.102"
MINIPC_WS   = "~/lunar_rover_ws"
RVIZ_CONFIG = os.path.expanduser("~/lunar_rover_ws/laptop_stream.rviz")

ACTION_DEFAULTS = {
    "drive_forward":     {"distance_m": 1.0, "speed": 120, "timeout_s": 60},
    "drive_backward":    {"distance_m": 0.5, "speed": 120, "timeout_s": 60},
    "pivot_turn":        {"degrees": 90.0,   "speed": 100, "timeout_s": 60},
    "actuator_position": {"target": "dig1",  "timeout_s": 12},
    "wait":              {"seconds": 1.0},
    "stop":              {},
}

# ─── Helpers ──────────────────────────────────────────────────────────────────

class LED(QLabel):
    _C = {"off": QColor(50,50,60), "green": QColor(50,210,80),
          "yellow": QColor(240,190,40), "red": QColor(210,60,60)}
    def __init__(self, c="off", parent=None):
        super().__init__(parent); self.setFixedSize(14,14); self._c = c
    def set_color(self, c): self._c = c; self.update()
    def paintEvent(self, e):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        c = self._C.get(self._c, self._C["off"])
        p.setBrush(QBrush(c)); p.setPen(QPen(c.darker(160), 1))
        p.drawEllipse(1,1,12,12)


def btn(text, bg, border, hover, h=34, w=None):
    b = QPushButton(text)
    b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
    b.setMinimumHeight(h)
    if w: b.setMinimumWidth(w)
    b.setStyleSheet(
        f"QPushButton{{background:{bg};color:#b0c8d8;border:1px solid {border};"
        f"border-radius:5px;padding:3px 10px;font-size:13px;font-weight:bold;}}"
        f"QPushButton:hover{{background:{border};color:#fff;}}"
        f"QPushButton:pressed{{background:{bg};}}")
    return b


# ─── ROS monitor ──────────────────────────────────────────────────────────────

class RosMonitor(QThread):
    mission_status = pyqtSignal(dict)
    encoder_raw = pyqtSignal(float)
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
            node.create_subscription(Float32, "/nav/encoder_raw",
                                     lambda m: self.encoder_raw.emit(float(m.data)), q)
            self._running = True
            from rclpy.executors import SingleThreadedExecutor
            ex = SingleThreadedExecutor(); ex.add_node(node)
            while self._running and rclpy.ok():
                ex.spin_once(timeout_sec=0.05)
        except Exception as e:
            print(f"[RosMonitor] {e}")
    def stop(self):
        self._running = False; self.quit(); self.wait(2000)


# ─── Step widget ──────────────────────────────────────────────────────────────

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
        lbl = QLabel(action.replace("_"," ").upper())
        lbl.setStyleSheet("color:#80b8d8;font-size:13px;font-weight:bold;")
        hr.addWidget(lbl); hr.addStretch()
        for sym, sig in [("^", self.sig_up), ("v", self.sig_down)]:
            b2 = QPushButton(sym); b2.setFixedSize(24,24)
            b2.setStyleSheet("background:#0e1018;color:#507080;border:1px solid #1c3040;"
                            "border-radius:3px;font-size:11px;font-weight:bold;")
            b2.clicked.connect(lambda _,s=sig: s.emit(self)); hr.addWidget(b2)
        xb = QPushButton("X"); xb.setFixedSize(24,24)
        xb.setStyleSheet("background:#1a0808;color:#c05050;border:1px solid #401010;"
                         "border-radius:3px;font-size:11px;font-weight:bold;")
        xb.clicked.connect(lambda: self.sig_remove.emit(self)); hr.addWidget(xb)
        root.addLayout(hr)
        if params:
            pl = QLabel("   ".join(f"{k}={v}" for k,v in params.items()))
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
        self.resize(1200, 880)
        self.setMinimumSize(960, 720)

        self._steps      = []
        self._yaml_path  = ""
        self._ros_pub_nd = None
        self._pub_ex     = None
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

        hrow = QHBoxLayout()
        ttl = QLabel("LUNAR ROVER  -  MISSION CONTROL")
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
        tg = QGroupBox("TELEOP  —  joystick via miniPC  (uses tmux, survives SSH disconnect)")
        tv = QVBoxLayout(tg); tv.setSpacing(5)
        tv.addWidget(QLabel(
            "L-stick Y = left wheels    R-stick Y = right wheels\n"
            "A=DIG2  Y=DRIVE  B=DIG1  X=CAL\n"
            "LT/RT = actuator hold (one direction each)    "
            "LB/RB = 360-servo hold (one direction each)\n"
            "Start = e-stop toggle"))
        joy_row = QHBoxLayout()
        self._joy_led = LED("off")
        self._joy_lbl = QLabel("not running")
        self._joy_lbl.setStyleSheet("color:#506070;font-size:12px;")
        joy_row.addWidget(self._joy_led); joy_row.addWidget(self._joy_lbl, 1)
        b_start = btn("START TELEOP","#081808","#1a5020","#28a040", h=40, w=200)
        b_start.clicked.connect(self._start_teleop); joy_row.addWidget(b_start)
        b_stop  = btn("STOP TELEOP", "#1a0808","#501010","#a02020", h=40, w=140)
        b_stop.clicked.connect(self._stop_teleop);  joy_row.addWidget(b_stop)
        tv.addLayout(joy_row)
        enc_row = QHBoxLayout()
        enc_row.addWidget(QLabel("Encoder distance/raw:"))
        self._enc_lbl = QLabel("--")
        self._enc_lbl.setStyleSheet("color:#50a0c8;font-weight:bold;")
        enc_row.addWidget(self._enc_lbl)
        enc_row.addStretch()
        tv.addLayout(enc_row)
        root.addWidget(tg)

        # ── QUICK DRIVE ────────────────────────────────────────────────────────
        dg = QGroupBox("QUICK DRIVE  —  straight distance  (via /nav/arduino_dist_cmd)")
        dv = QVBoxLayout(dg); dv.setSpacing(5)
        dr = QHBoxLayout()
        for lbl, m in [("0.5m FWD",0.5),("1m FWD",1.0),("2m FWD",2.0),
                       ("0.5m REV",-0.5),("1m REV",-1.0)]:
            b2 = btn(lbl,"#0e1018","#1c3848","#2a6060",h=36,w=90)
            b2.clicked.connect(lambda _,v=m: self._send_dist(v)); dr.addWidget(b2)
        dr.addStretch()
        b_stop_all = btn("STOP ALL","#1a0808","#601010","#c02020",h=36,w=110)
        b_stop_all.clicked.connect(lambda: self._send_raw_cmd(0xFF,0,0,0))
        dr.addWidget(b_stop_all)
        dv.addLayout(dr)

        cr = QHBoxLayout()
        cr.addWidget(QLabel("Custom (m):"))
        self._dist_edit = QLineEdit("1.0"); self._dist_edit.setFixedWidth(80)
        cr.addWidget(self._dist_edit)
        cr.addWidget(btn("FWD","#0e1820","#1a4060","#2a80c0",h=32,w=60,
                         ).also(lambda b: b.clicked.connect(lambda: self._send_custom(1))))
        cr.addWidget(btn("REV","#1a0808","#402010","#804020",h=32,w=60,
                         ).also(lambda b: b.clicked.connect(lambda: self._send_custom(-1))))
        cr.addStretch()
        dv.addLayout(cr)
        root.addWidget(dg)

        # ── TURN CONTROL ───────────────────────────────────────────────────────
        trn = QGroupBox("TURN CONTROL  —  encoder-based  (via /nav/arduino_turn_cmd)")
        tv2 = QVBoxLayout(trn); tv2.setSpacing(6)

        # Speed slider
        spd_row = QHBoxLayout()
        spd_row.addWidget(QLabel("Turn speed (PWM 1-190):"))
        self._turn_spd_slider = QSlider(Qt.Horizontal)
        self._turn_spd_slider.setRange(20, 190); self._turn_spd_slider.setValue(100)
        self._turn_spd_slider.setFixedHeight(24)
        self._turn_spd_lbl = QLabel("100")
        self._turn_spd_lbl.setFixedWidth(36)
        self._turn_spd_lbl.setStyleSheet("color:#50a0c8;font-weight:bold;")
        self._turn_spd_slider.valueChanged.connect(
            lambda v: self._turn_spd_lbl.setText(str(v)))
        spd_row.addWidget(self._turn_spd_slider,1); spd_row.addWidget(self._turn_spd_lbl)
        tv2.addLayout(spd_row)

        # Point turn buttons (same distance both sides, opposite direction)
        pt_lbl = QLabel("Point turn (pivot):")
        pt_lbl.setStyleSheet("color:#8090a8;")
        tv2.addWidget(pt_lbl)
        pt_row = QHBoxLayout()
        for lbl, deg in [("90° CCW",90),("45° CCW",45),("30° CCW",30),
                          ("30° CW",-30),("45° CW",-45),("90° CW",-90)]:
            b2 = btn(lbl,"#0e1820","#1a3858","#2a5878",h=34)
            b2.clicked.connect(lambda _,d=deg: self._send_pivot(d))
            pt_row.addWidget(b2)
        tv2.addLayout(pt_row)

        # Custom point turn
        pt2 = QHBoxLayout()
        pt2.addWidget(QLabel("Custom degrees:"))
        self._pt_deg = QDoubleSpinBox()
        self._pt_deg.setRange(-360,360); self._pt_deg.setValue(90); self._pt_deg.setDecimals(1)
        self._pt_deg.setFixedWidth(90)
        pt2.addWidget(self._pt_deg)
        pt2.addWidget(QLabel("(+ = CCW,  - = CW)"))
        b_pt = btn("PIVOT TURN","#0e1820","#2a4060","#4080c0",h=34,w=140)
        b_pt.clicked.connect(lambda: self._send_pivot(self._pt_deg.value()))
        pt2.addWidget(b_pt); pt2.addStretch()
        tv2.addLayout(pt2)

        # Arc turn: different distances per side
        arc_lbl = QLabel("Arc turn  (one side farther than the other):")
        arc_lbl.setStyleSheet("color:#8090a8;")
        tv2.addWidget(arc_lbl)

        arc_row = QHBoxLayout()
        arc_row.addWidget(QLabel("L wheel mm:"))
        self._arc_bl = QDoubleSpinBox()
        self._arc_bl.setRange(-5000,5000); self._arc_bl.setValue(500)
        self._arc_bl.setDecimals(0); self._arc_bl.setFixedWidth(90)
        arc_row.addWidget(self._arc_bl)
        arc_row.addWidget(QLabel("  R wheel mm:"))
        self._arc_br = QDoubleSpinBox()
        self._arc_br.setRange(-5000,5000); self._arc_br.setValue(300)
        self._arc_br.setDecimals(0); self._arc_br.setFixedWidth(90)
        arc_row.addWidget(self._arc_br)
        arc_row.addWidget(QLabel("  (+ = fwd, - = rev for each wheel)"))
        b_arc = btn("RUN ARC","#0e1820","#2a4060","#4080c0",h=34,w=120)
        b_arc.clicked.connect(self._send_arc)
        arc_row.addWidget(b_arc); arc_row.addStretch()
        tv2.addLayout(arc_row)

        arc_hint = QLabel(
            "Presets:  "
            "L=500 R=300 → gentle right curve  |  "
            "L=500 R=0 → sharp right (inner stops)  |  "
            "L=500 R=-500 → pivot right")
        arc_hint.setStyleSheet("color:#3a5870;font-size:11px;")
        tv2.addWidget(arc_hint)
        root.addWidget(trn)

        # ── ACTUATOR ───────────────────────────────────────────────────────────
        ag = QGroupBox("ACTUATOR")
        av = QHBoxLayout(ag)
        for lbl, cmd in [("DIG 1",0xA7),("DIG 2",0x93),("DRIVE POS",0xA9),("CALIBRATE",0xCA)]:
            b2 = btn(lbl,"#1a1028","#3a1060","#7030c0",h=36,w=110)
            b2.clicked.connect(lambda _,c=cmd: self._send_raw_cmd(c,0,0,0))
            av.addWidget(b2)
        av.addStretch()
        root.addWidget(ag)

        # ── STACK / RVIZ ───────────────────────────────────────────────────────
        sg = QGroupBox("STACK LAUNCH  —  SSH to miniPC")
        sv = QHBoxLayout(sg)
        for lbl, fn in [("Autonomous stack",self._launch_auto),
                        ("Full miniPC stack",self._launch_full)]:
            b2 = btn(lbl,"#0e1820","#1a4060","#2a80c0",h=36)
            b2.clicked.connect(fn); sv.addWidget(b2)
        sv.addWidget(btn("Open RViz2 (local)","#0e1018","#1a3040","#2a5060",h=36,
                         ).also(lambda b: b.clicked.connect(self._launch_rviz)))
        sv.addStretch()
        root.addWidget(sg)
        root.addStretch()
        return w

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
        for lbl, fn in [("Load",self._quick_load),("Refresh",self._refresh_yamls),
                        ("Browse",self._browse_yaml)]:
            b2 = btn(lbl,"#0e1820","#1a4060","#2a80c0",h=28,w=80)
            b2.clicked.connect(fn); ql.addWidget(b2)
        root.addLayout(ql)
        self._file_lbl = QLabel("No file loaded")
        self._file_lbl.setStyleSheet("color:#405868;font-size:12px;")
        root.addWidget(self._file_lbl)

        # Note about mission.yaml
        note = QLabel(
            "NOTE: Edit mission.yaml before running — check distance_m values are in METRES not mm.\n"
            "The 'Excavation run' example has distance_m:1436 (1436 metres!) — change to e.g. 1.436")
        note.setStyleSheet(
            "color:#906030;font-size:11px;background:#1a1208;"
            "border:1px solid #604020;border-radius:4px;padding:4px 8px;")
        note.setWordWrap(True)
        root.addWidget(note)

        spl = QSplitter(Qt.Horizontal)
        spl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Left: step list
        lw = QWidget(); lv = QVBoxLayout(lw); lv.setContentsMargins(0,0,4,0)
        lv.addWidget(QLabel("Mission steps:"))
        self._scroll = QScrollArea(); self._scroll.setWidgetResizable(True)
        self._scroll.setStyleSheet(
            "QScrollArea{background:#080b10;border:1px solid #182030;}")
        self._step_box = QWidget(); self._step_lay = QVBoxLayout(self._step_box)
        self._step_lay.setSpacing(3); self._step_lay.setAlignment(Qt.AlignTop)
        self._scroll.setWidget(self._step_box); lv.addWidget(self._scroll,1)
        lr2 = QHBoxLayout()
        for lbl, fn in [("Clear all",self._clear_steps),("Save YAML",self._save_yaml)]:
            b2 = btn(lbl,"#0e1018","#1a2030","#2a3848",h=28); b2.clicked.connect(fn)
            lr2.addWidget(b2)
        lv.addLayout(lr2); spl.addWidget(lw)

        # Right: add-step panel
        rw = QWidget(); rv = QVBoxLayout(rw)
        rv.setContentsMargins(4,0,0,0); rv.setSpacing(6)
        rv.addWidget(QLabel("Add step:"))
        self._act_combo = QComboBox()
        self._act_combo.addItems(list(ACTION_DEFAULTS.keys()))
        self._act_combo.currentTextChanged.connect(self._refresh_params)
        rv.addWidget(self._act_combo)
        self._param_area = QWidget(); self._param_lay = QVBoxLayout(self._param_area)
        self._param_lay.setSpacing(4); rv.addWidget(self._param_area)
        self._param_wids = {}
        self._refresh_params(self._act_combo.currentText())
        ab = btn("+ Add Step","#0e1820","#1a4060","#2a80c0",h=36)
        ab.clicked.connect(self._add_step); rv.addWidget(ab)
        rv.addStretch(); spl.addWidget(rw)
        spl.setSizes([420,300]); root.addWidget(spl,1)

        nr = QHBoxLayout(); nr.addWidget(QLabel("Name:"))
        self._name_edit = QLineEdit("Untitled Mission"); nr.addWidget(self._name_edit,1)
        root.addLayout(nr)

        rr = QHBoxLayout()
        self._run_btn   = btn("START MISSION","#081a08","#1a5020","#28a040",h=42,w=220)
        self._abort_btn = btn("ABORT",        "#1a0808","#501010","#a02020",h=42,w=110)
        self._run_btn.clicked.connect(self._start_mission)
        self._abort_btn.clicked.connect(self._abort_mission)
        rr.addWidget(self._run_btn); rr.addWidget(self._abort_btn); rr.addStretch()
        root.addLayout(rr)

        self._mstatus = QLabel("Idle — load or build a mission then click START MISSION")
        self._mstatus.setStyleSheet("color:#405868;font-size:12px;")
        root.addWidget(self._mstatus)
        self._mprog = QProgressBar(); self._mprog.setFixedHeight(6)
        self._mprog.setTextVisible(False)
        self._mprog.setStyleSheet(
            "QProgressBar{background:#0e1218;border:none;border-radius:3px;}"
            "QProgressBar::chunk{background:#28a040;border-radius:3px;}")
        root.addWidget(self._mprog)
        self._refresh_yamls()
        return w

    # ═════════════════════════════════════════════════════════════════════════
    # LOG TAB
    # ═════════════════════════════════════════════════════════════════════════

    def _tab_log(self):
        w = QWidget(); root = QVBoxLayout(w); root.setContentsMargins(10,10,10,10)
        self.log_view = QTextEdit(); self.log_view.setReadOnly(True)
        root.addWidget(self.log_view,1)
        cb = btn("Clear log","#0e1018","#1a2030","#2a3040",h=28,w=90)
        cb.clicked.connect(self.log_view.clear); root.addWidget(cb)
        return w

    # ═════════════════════════════════════════════════════════════════════════
    # PARAM EDITOR
    # ═════════════════════════════════════════════════════════════════════════

    def _refresh_params(self, action):
        while self._param_lay.count():
            item = self._param_lay.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        self._param_wids.clear()
        for key, default in ACTION_DEFAULTS.get(action,{}).items():
            row = QHBoxLayout()
            lbl = QLabel(f"{key}:")
            lbl.setStyleSheet("color:#6080a0;font-size:12px;min-width:120px;")
            row.addWidget(lbl)
            if key == "target":
                cb = QComboBox(); cb.addItems(["dig1", "dig2", "drive", "calibrate"])
                cb.setCurrentText(str(default)); cb.setFixedHeight(28)
                row.addWidget(cb); self._param_wids[key] = cb
            elif isinstance(default, bool):
                ck = QCheckBox(); ck.setChecked(default)
                row.addWidget(ck); self._param_wids[key] = ck
            elif isinstance(default, float):
                sb = QDoubleSpinBox(); sb.setDecimals(3); sb.setRange(-9999,9999)
                sb.setValue(default); sb.setFixedHeight(28)
                row.addWidget(sb); self._param_wids[key] = sb
            elif isinstance(default, int):
                sb = QSpinBox(); sb.setRange(0,9999); sb.setValue(default)
                sb.setFixedHeight(28); row.addWidget(sb); self._param_wids[key] = sb
            else:
                le = QLineEdit(str(default)); le.setFixedHeight(28)
                row.addWidget(le); self._param_wids[key] = le
            self._param_lay.addLayout(row)

    def _read_params(self):
        action = self._act_combo.currentText(); params = {}
        for key, default in ACTION_DEFAULTS.get(action,{}).items():
            w = self._param_wids.get(key)
            if not w: continue
            if isinstance(w, QCheckBox):                  params[key] = w.isChecked()
            elif isinstance(w, (QDoubleSpinBox,QSpinBox)): params[key] = w.value()
            elif isinstance(w, QComboBox):                params[key] = w.currentText()
            else:
                raw = w.text().strip()
                try:
                    if isinstance(default, float): params[key] = float(raw)
                    elif isinstance(default, int): params[key] = int(raw)
                    else: params[key] = raw
                except: params[key] = default
        return params

    # ═════════════════════════════════════════════════════════════════════════
    # STEP MANAGEMENT
    # ═════════════════════════════════════════════════════════════════════════

    def _add_step(self):
        self._insert(self._act_combo.currentText(), self._read_params())

    def _insert(self, action, params):
        sw = StepWidget(action, params)
        sw.sig_remove.connect(self._rm); sw.sig_up.connect(self._mu)
        sw.sig_down.connect(self._md)
        self._steps.append(sw); self._step_lay.addWidget(sw)
        self._mstatus.setText(f"{len(self._steps)} step(s)")

    def _rm(self, sw):
        if sw in self._steps:
            self._steps.remove(sw); self._step_lay.removeWidget(sw); sw.deleteLater()
            self._mstatus.setText(f"{len(self._steps)} step(s)")

    def _mu(self, sw):
        i = self._steps.index(sw)
        if i > 0:
            self._steps[i], self._steps[i-1] = self._steps[i-1], self._steps[i]
            self._rebuild()

    def _md(self, sw):
        i = self._steps.index(sw)
        if i < len(self._steps)-1:
            self._steps[i], self._steps[i+1] = self._steps[i+1], self._steps[i]
            self._rebuild()

    def _rebuild(self):
        for sw in self._steps: self._step_lay.removeWidget(sw)
        for sw in self._steps: self._step_lay.addWidget(sw)

    def _clear_steps(self):
        for sw in list(self._steps): self._rm(sw)
        self._yaml_path = ""; self._file_lbl.setText("No file loaded")

    def _mission_dict(self):
        return {"mission":{"name":self._name_edit.text(),
                           "steps":[sw.to_dict() for sw in self._steps]}}

    # ═════════════════════════════════════════════════════════════════════════
    # YAML I/O
    # ═════════════════════════════════════════════════════════════════════════

    def _refresh_yamls(self):
        self._yaml_combo.clear()
        ws = os.path.expanduser("~/lunar_rover_ws")
        files = sorted(glob.glob(f"{ws}/*.yaml") + glob.glob(f"{ws}/*.yml"))
        for f in files: self._yaml_combo.addItem(Path(f).name, f)
        if not files: self._yaml_combo.addItem("(none found)", "")

    def _quick_load(self):
        p = self._yaml_combo.currentData()
        if p and os.path.exists(p): self._load(p)
        else: self._log("No valid YAML selected in dropdown")

    def _browse_yaml(self):
        p,_ = QFileDialog.getOpenFileName(
            self,"Load YAML",os.path.expanduser("~/lunar_rover_ws"),"YAML (*.yaml *.yml)")
        if p: self._load(p)

    def _load(self, path):
        if not YAML_AVAILABLE:
            self._log("pyyaml missing: pip3 install pyyaml --break-system-packages"); return
        try:
            with open(path) as f: doc = yaml.safe_load(f)
            mission = doc.get("mission",doc)
            steps = mission.get("steps",[]); name = mission.get("name",Path(path).stem)
            self._clear_steps(); self._name_edit.setText(name)
            for s in steps: self._insert(s.get("action","stop"), s.get("params",{}))
            self._yaml_path = path
            self._file_lbl.setText(f"Loaded: {Path(path).name}  ({len(steps)} steps)")
            self._log(f"Loaded {len(steps)} steps from {Path(path).name}")
        except Exception as e:
            self._log(f"YAML load error: {e}")

    def _save_yaml(self):
        if not YAML_AVAILABLE: self._log("pyyaml missing"); return
        default = self._yaml_path or os.path.expanduser("~/lunar_rover_ws/mission.yaml")
        path,_ = QFileDialog.getSaveFileName(self,"Save YAML",default,"YAML (*.yaml *.yml)")
        if not path: return
        try:
            with open(path,"w") as f:
                yaml.dump(self._mission_dict(),f,default_flow_style=False,sort_keys=False)
            self._yaml_path = path
            self._file_lbl.setText(f"Saved: {Path(path).name}")
            self._log(f"Saved to {path}"); self._refresh_yamls()
        except Exception as e:
            self._log(f"Save error: {e}")

    # ═════════════════════════════════════════════════════════════════════════
    # MISSION RUN
    # ═════════════════════════════════════════════════════════════════════════

    def _ros2_pub_cmd(self, topic, msg_type, value_str):
        """Publish via ros2 topic pub subprocess — same method as run_mission.sh."""
        ws_setup = os.path.expanduser("~/lunar_rover_ws/install/setup.bash")
        ros_env = (
            "source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null; "
            + (f"source {ws_setup} 2>/dev/null; " if os.path.exists(ws_setup) else "")
            + "export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 "
            "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; "
            f"ros2 topic pub --once {topic} {msg_type} \"{value_str}\""
        )
        r = subprocess.run(["bash", "-c", ros_env],
                           capture_output=True, text=True, timeout=12)
        return r.returncode == 0, r.stdout + r.stderr

    def _ensure_pubs(self):
        """Keep for quick-command ROS publishers (drive/cmd buttons)."""
        if not ROS_AVAILABLE or self._start_pub is not None: return
        try:
            if not rclpy.ok(): rclpy.init()
            self._ros_pub_nd = rclpy.create_node("rover_gui_pub")
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
            self._start_pub = self._ros_pub_nd.create_publisher(Bool,   "/mission/start",        q)
            self._file_pub  = self._ros_pub_nd.create_publisher(String, "/mission/file",          q)
            self._dist_pub  = self._ros_pub_nd.create_publisher(Float32,"/nav/arduino_dist_cmd",  q)
            self._cmd_pub   = self._ros_pub_nd.create_publisher(
                Float32MultiArray, "/nav/arduino_cmd",  q)
            self._turn_pub  = self._ros_pub_nd.create_publisher(
                Float32MultiArray, "/nav/arduino_turn_cmd", q)
            from rclpy.executors import SingleThreadedExecutor
            self._pub_ex = SingleThreadedExecutor()
            self._pub_ex.add_node(self._ros_pub_nd)
            for _ in range(5):
                self._pub_ex.spin_once(timeout_sec=0.02)
            self._log("ROS publishers ready")
        except Exception as e:
            self._log(f"ROS publisher init failed: {e}")

    def _flush_ros(self, spins=2):
        """Give DDS a moment to actually transmit outgoing messages."""
        if self._pub_ex is None:
            return
        for _ in range(max(1, spins)):
            self._pub_ex.spin_once(timeout_sec=0.01)

    def _ensure_remote_mission_stack(self):
        """Start bridge+sequencer on miniPC if they are not already up."""
        remote = (
            'source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null\n'
            f'[ -f {MINIPC_WS}/install/setup.bash ] && source {MINIPC_WS}/install/setup.bash\n'
            'export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET\n'
            'ros2 node list 2>/dev/null | grep -q "/nav_arduino_bridge" || '
            f'(nohup python3 {MINIPC_WS}/nav_arduino_bridge.py >/tmp/rover_bridge.log 2>&1 &)\n'
            'ros2 node list 2>/dev/null | grep -q "/nav_mission_sequencer" || '
            f'(nohup python3 {MINIPC_WS}/nav_mission_sequencer.py >/tmp/rover_sequencer.log 2>&1 &)\n'
            'sleep 1\n'
            'echo STACK_READY\n'
        )
        try:
            r = subprocess.run(
                ["ssh", "-o", "ConnectTimeout=8", "-o", "StrictHostKeyChecking=no",
                 f"{MINIPC_USER}@{MINIPC_IP}", "bash", "-s"],
                input=remote, capture_output=True, text=True, timeout=15)
            return ("STACK_READY" in (r.stdout or "")), (r.stdout + r.stderr)
        except Exception as e:
            return False, str(e)

    def _start_mission(self):
        if not self._steps:
            self._log("No steps — add steps or load a YAML first"); return

        # Save to laptop first
        local_tmp = os.path.expanduser("~/lunar_rover_ws/_gui_mission.yaml")
        remote_tmp = f"{MINIPC_WS}/_gui_mission.yaml"  # path ON miniPC

        if not YAML_AVAILABLE:
            self._log("pyyaml missing — cannot auto-save"); return
        try:
            os.makedirs(os.path.dirname(local_tmp), exist_ok=True)
            with open(local_tmp,"w") as f:
                yaml.dump(self._mission_dict(),f,default_flow_style=False,sort_keys=False)
            self._log(f"Saved locally: {local_tmp}")
        except Exception as e:
            self._log(f"Auto-save failed: {e}"); return

        def _send():
            ok_stack, out_stack = self._ensure_remote_mission_stack()
            if not ok_stack:
                self._log("Could not start mission stack on miniPC.")
                self._log(f"  {out_stack[:250]}")
                self._log("  Run manually: bash ~/lunar_rover_ws/run_mission.sh --dry-run mission.yaml")
                return

            # Step 1: SCP the YAML to the miniPC
            self._log(f"Copying YAML to miniPC...")
            scp = subprocess.run(
                ["scp", "-o", "ConnectTimeout=8", "-o", "StrictHostKeyChecking=no",
                 local_tmp, f"{MINIPC_USER}@{MINIPC_IP}:{remote_tmp}"],
                capture_output=True, text=True, timeout=15)
            if scp.returncode != 0:
                self._log(f"SCP failed: {scp.stderr or scp.stdout}")
                self._log("Cannot send YAML to miniPC. Is SSH set up?")
                return
            self._log("YAML copied to miniPC OK")

            # Step 2: Send /mission/file with the miniPC path, then /mission/start
            self._log("Sending /mission/file ...")
            ok, out = self._ros2_pub_cmd(
                "/mission/file", "std_msgs/msg/String", f"data: '{remote_tmp}'")
            self._log(f"  file: {'OK' if ok else 'FAILED'}")
            if not ok:
                self._log(f"  {out[:200]}")
                self._log("  Is nav_mission_sequencer.py running on miniPC?")
                self._log("  Start it: ssh cheese@miniPC 'python3 ~/lunar_rover_ws/nav_mission_sequencer.py &'")
                return
            time.sleep(0.5)
            self._log("Sending /mission/start ...")
            ok2, out2 = self._ros2_pub_cmd(
                "/mission/start", "std_msgs/msg/Bool", "data: true")
            self._log(f"  start: {'OK — mission running!' if ok2 else 'FAILED'}")
            if not ok2:
                self._log(f"  {out2[:200]}")
        threading.Thread(target=_send, daemon=True).start()
        self._mstatus.setText("Mission starting..."); self._mprog.setValue(5)

    def _abort_mission(self):
        def _send():
            ok, _ = self._ros2_pub_cmd(
                "/mission/start", "std_msgs/msg/Bool", "data: false")
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
            self._flush_ros(spins=3)
            self._log(f"Drive {m:+.1f}m")
        else: self._log("ROS unavailable — is nav_arduino_bridge running on miniPC?")

    def _send_custom(self, sign):
        try: self._send_dist(float(self._dist_edit.text())*sign)
        except ValueError: self._log("Invalid distance value")

    def _send_raw_cmd(self, device, speed, direction, lobyte):
        self._ensure_pubs()
        if ROS_AVAILABLE and self._cmd_pub:
            m = Float32MultiArray()
            m.data = [float(device),float(speed),float(direction),float(lobyte)]
            self._cmd_pub.publish(m)
            time.sleep(0.02)
            self._cmd_pub.publish(m)  # duplicate once for lossy Wi-Fi links
            self._flush_ros(spins=3)
            self._log(f"Cmd 0x{device:02X} sp={speed} dir={direction}")
        else: self._log("ROS unavailable")

    def _send_pivot(self, degrees: float):
        """Send a pivot turn via /nav/arduino_turn_cmd [arc_mm, speed, clockwise_int]"""
        speed = self._turn_spd_slider.value()
        # arc_mm = (700/2) * |radians| — uses default track width
        arc_mm = 350.0 * abs(degrees * 3.14159265 / 180.0)
        cw = 1 if degrees < 0 else 0   # negative degrees = CW
        self._ensure_pubs()
        if ROS_AVAILABLE and self._turn_pub:
            m = Float32MultiArray()
            m.data = [arc_mm, float(speed), float(cw)]
            self._turn_pub.publish(m)
            self._flush_ros(spins=3)
            # Bridge/firmware combos may keep pivot mode active until STOPALL.
            # Send an automatic stop after estimated turn time.
            speed_ms = max(0.20 * (speed / 120.0), 0.05)
            turn_s = ((arc_mm / 1000.0) / speed_ms) * 1.3 + 0.6
            threading.Thread(
                target=lambda: (time.sleep(turn_s), self._send_raw_cmd(0xFF, 0, 0, 0)),
                daemon=True
            ).start()
            self._log(f"Pivot {degrees:+.1f}°  arc={arc_mm:.0f}mm  "
                      f"{'CW' if cw else 'CCW'}  speed={speed}")
        else: self._log("ROS unavailable")

    def _send_arc(self):
        """Send an arc turn using raw C8/C9/E8 via /nav/arduino_cmd sequence."""
        bl_mm = self._arc_bl.value()
        br_mm = self._arc_br.value()
        speed = self._turn_spd_slider.value()
        self._ensure_pubs()
        if not ROS_AVAILABLE or self._cmd_pub is None:
            self._log("ROS unavailable"); return

        def _enc(mm, rev=False):
            u = int(min(0x7FFF, max(0, round(abs(mm)))))
            c = ((1 if rev else 0) << 15) | u
            return (c >> 8) & 0xFF, c & 0xFF

        bl_db, bl_lo = _enc(abs(bl_mm), bl_mm < 0)
        br_db, br_lo = _enc(abs(br_mm), br_mm < 0)

        def _send_seq():
            m = Float32MultiArray()
            # Stop any existing motion first to avoid stale turn state.
            m.data = [0xFF, 0.0, 0.0, 0.0]
            self._cmd_pub.publish(m); time.sleep(0.04)
            # Load BL
            m.data = [0xC8, float(speed), float(bl_db), float(bl_lo)]
            self._cmd_pub.publish(m); time.sleep(0.04)
            # Load BR
            m.data = [0xC9, float(speed), float(br_db), float(br_lo)]
            self._cmd_pub.publish(m); time.sleep(0.04)
            # Start isolated
            m.data = [0xE8, 0.0, 0.0, 0.0]
            self._cmd_pub.publish(m)
            # Auto-stop arc after estimated completion.
            longest_mm = max(abs(bl_mm), abs(br_mm))
            speed_ms = max(0.20 * (speed / 120.0), 0.05)
            turn_s = ((longest_mm / 1000.0) / speed_ms) * 1.3 + 0.6
            time.sleep(turn_s)
            m.data = [0xFF, 0.0, 0.0, 0.0]
            self._cmd_pub.publish(m)
            self._log(f"Arc  BL={bl_mm:+.0f}mm  BR={br_mm:+.0f}mm  speed={speed}")
        threading.Thread(target=_send_seq, daemon=True).start()

    # ═════════════════════════════════════════════════════════════════════════
    # TELEOP  —  uses tmux so processes survive SSH disconnect
    # ═════════════════════════════════════════════════════════════════════════

    def _start_teleop(self):
        self._log("SSH → miniPC: launching joy_node + joy_to_arduino...")
        self._joy_led.set_color("yellow")
        self._joy_lbl.setText("starting...")

        # This is the exact pattern from the original working GUI.
        # Key requirements:
        #   1. Use 'disown -a' so SSH exits immediately after launching background jobs
        #   2. Wait between joy_node start and joy_to_arduino start (3s)
        #   3. Use echo LAUNCH_DONE as sentinel that SSH completed successfully
        # DO NOT poll /joy topic — that blocks SSH for too long.
        remote_script = (
            'source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null\n'
            f'[ -f {MINIPC_WS}/install/setup.bash ] && source {MINIPC_WS}/install/setup.bash\n'
            'export ROS_DOMAIN_ID=42\n'
            'export ROS_LOCALHOST_ONLY=0\n'
            'export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET\n'
            # Kill any old instances
            'pkill -f joy_to_arduino 2>/dev/null\n'
            'pkill -f joy_node        2>/dev/null\n'
            'sleep 1\n'
            # Launch joy_node in background, log to file
            'ros2 run joy joy_node > /tmp/joy_node.log 2>&1 &\n'
            'sleep 3\n'
            # Launch joy_to_arduino in background, log to file
            f'python3 {MINIPC_WS}/joy_to_arduino.py > /tmp/joy_arduino.log 2>&1 &\n'
            'sleep 1\n'
            # Detach all background jobs so SSH exits cleanly
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
                self._log(f"  SSH output: {stdout.strip()[:200]}")
                if "LAUNCH_DONE" in stdout:
                    self._joy_led.set_color("green")
                    self._joy_lbl.setText("running on miniPC")
                    self._log("Teleop started. Controller should respond in ~3s.")
                    self._log("  Check logs: ssh cheese@192.168.0.102 cat /tmp/joy_node.log")
                    self._log("  Check logs: ssh cheese@192.168.0.102 cat /tmp/joy_arduino.log")
                else:
                    self._joy_led.set_color("red")
                    self._joy_lbl.setText("failed — see LOG tab")
                    self._log("Teleop launch failed. Check:")
                    self._log("  1. SSH keys work: ssh cheese@192.168.0.102 echo ok")
                    self._log("  2. joy package: ros2 pkg list | grep joy")
                    self._log(f"  Full output: {stdout[:400]}")
            except subprocess.TimeoutExpired:
                self._joy_led.set_color("red")
                self._joy_lbl.setText("SSH timeout")
                self._log("SSH timed out after 20s — check miniPC connection")
            except Exception as e:
                self._joy_led.set_color("red")
                self._joy_lbl.setText("error")
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
                stdout, _ = proc.communicate(input=remote_script, timeout=12)
                self._joy_led.set_color("off")
                self._joy_lbl.setText("stopped")
                self._log("Teleop stopped")
            except Exception as e:
                self._log(f"Stop teleop error: {e}")
        threading.Thread(target=run, daemon=True).start()

    # ═════════════════════════════════════════════════════════════════════════
    # STACK LAUNCH / RVIZ
    # ═════════════════════════════════════════════════════════════════════════

    def _ssh_bg(self, cmd, label):
        env = ("source /opt/ros/$(ls /opt/ros | head -1)/setup.bash 2>/dev/null; "
               f"source {MINIPC_WS}/install/setup.bash 2>/dev/null; "
               "export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 "
               "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; ")
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
        self._ssh_bg(
            f"nohup bash {MINIPC_WS}/full_launch_autonomous.sh "
            f"> /tmp/minipc_auto.log 2>&1 &", "auto_stack")

    def _launch_full(self):
        self._log("Starting full miniPC stack...")
        self._ssh_bg(
            f"nohup bash {MINIPC_WS}/full_launch_minipc.sh "
            f"> /tmp/minipc_launch.log 2>&1 &", "full_stack")

    def _launch_rviz(self):
        cmd = ["rviz2", "-d", RVIZ_CONFIG] if os.path.exists(RVIZ_CONFIG) else ["rviz2"]
        try:
            subprocess.Popen(cmd)
            self._log(f"RViz2 launched locally")
        except FileNotFoundError:
            self._log("rviz2 not found — source ROS2 on the laptop first")

    # ═════════════════════════════════════════════════════════════════════════
    # ROS MONITOR
    # ═════════════════════════════════════════════════════════════════════════

    def _start_ros_monitor(self):
        self._ros_mon = RosMonitor()
        self._ros_mon.mission_status.connect(self._on_mission_status)
        self._ros_mon.encoder_raw.connect(self._on_encoder_raw)
        self._ros_mon.start()

    @pyqtSlot(dict)
    def _on_mission_status(self, d):
        step  = d.get("step",0); total = d.get("total",0)
        name  = d.get("step_name",""); run = d.get("running",False)
        if total > 0: self._mprog.setValue(int(step/total*100))
        self._mstatus.setText(
            f"Step {step+1}/{total}: {name}" if run
            else ("Complete" if step > 0 and total > 0 else "Idle"))

    @pyqtSlot(float)
    def _on_encoder_raw(self, val):
        self._enc_lbl.setText(f"{val:.2f}")

    # ═════════════════════════════════════════════════════════════════════════
    # CONN CHECK + LOG
    # ═════════════════════════════════════════════════════════════════════════

    def _start_conn_check(self):
        self._ct = QTimer(); self._ct.timeout.connect(self._check_conn)
        self._ct.start(8000); self._check_conn()

    def _check_conn(self):
        def run():
            r = subprocess.run(f"ping -c1 -W2 {MINIPC_IP}",
                               shell=True, capture_output=True)
            ok = r.returncode == 0
            self._conn_led.set_color("green" if ok else "red")
            self._conn_lbl.setText(
                f"miniPC {MINIPC_IP}: {'online' if ok else 'offline'}")
        threading.Thread(target=run, daemon=True).start()

    def _log(self, msg):
        self._log_sig.emit(str(msg))

    def _log_direct(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.append(f"<span style='color:#304858'>[{ts}]</span> {msg}")

    def closeEvent(self, e):
        if hasattr(self,"_ros_mon"): self._ros_mon.stop()
        e.accept()


# ─── Button helper — .also() for one-liner wiring ────────────────────────────

class _BtnProxy(QPushButton):
    def also(self, fn):
        fn(self); return self

def btn(text, bg, border, hover, h=34, w=None):
    b = _BtnProxy(text)
    b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
    b.setMinimumHeight(h)
    if w: b.setMinimumWidth(w)
    b.setStyleSheet(
        f"QPushButton{{background:{bg};color:#b0c8d8;border:1px solid {border};"
        f"border-radius:5px;padding:3px 10px;font-size:13px;font-weight:bold;}}"
        f"QPushButton:hover{{background:{border};color:#fff;}}"
        f"QPushButton:pressed{{background:{bg};}}")
    return b


def main():
    app = QApplication(sys.argv)
    app.setFont(QFont("Courier New", 13))
    w = GUI(); w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
