#!/usr/bin/env python3
"""
nav_control_panel.py  —  runs on LAPTOP

KEY DESIGN:  Lucas-Kanade optical flow on RGB for feature tracking
═══════════════════════════════════════════════════════════════════

PROBLEM WITH FIXED PIXELS
  If we store a goal as pixel (u=200, v=150) and the rover moves
  forward, pixel (200,150) now shows a different part of the ground.
  The goal appears to "slide" forward.

  Storing the 3D camera-frame point and re-projecting it each frame
  also fails because we need the camera-to-world transform which
  requires accurate odometry.

THE CORRECT APPROACH: track in the RGB image
  1. When user clicks pixel (u,v) on the depth image, read depth
     there to get 3D point.  Also record the GRAYSCALE patch from
     the RGB image at that pixel.

  2. Each new RGB frame, run cv2.calcOpticalFlowPyrLK() to find
     where that point has moved to in image space.  LK works on
     actual image texture — rock edges, shadows, regolith patterns.

  3. At the new tracked pixel, read depth from the depth image.
     Send this updated 3D position to the miniPC.

  4. Draw all markers at the TRACKED pixel, not the original pixel.

REGOLITH CAVEAT
  Regolith has low texture.  LK will work on:
    - Rock edges (strong gradient, trackable)
    - Shadow boundaries
    - Any surface colour variation
  If tracking confidence drops (error > threshold), we fall back to
  the last good position and flash a warning.  User can re-click.

  For completely featureless ground (flat sand, no rocks) tracking
  will fail.  In that case the goal appears at the last known pixel.
  This is no worse than the fixed-pixel approach.

PATH OVERLAY
  Path waypoints come from miniPC in base_link (robot-local) frame.
  We project each (fwd, left) ground point into the camera image
  using the known tilt geometry.  The green line lies on the ground.

AUTO OBSTACLE OVERLAY
  Obstacle markers from miniPC (base_link frame) are projected into
  the depth image the same way.  Red boxes show detected rocks.
"""

import sys
import math
import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage, CameraInfo
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Bool, String
from visualization_msgs.msg import MarkerArray

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QHBoxLayout, QVBoxLayout, QFrame, QSizePolicy, QGridLayout, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread, QRect
from PyQt5.QtGui import QImage, QPixmap, QFont


# ═══════════════════════════════════════════════════════════════
#  DEPTH DECODING  (turbo colourmap → metres)
# ═══════════════════════════════════════════════════════════════

_TURBO_N   = 4001
_TURBO_MAX = 4.0   # metres

def _make_turbo_lut():
    idx = np.arange(_TURBO_N, dtype=np.float32).reshape(1, _TURBO_N, 1)
    u8  = (idx / (_TURBO_N - 1) * 255).astype(np.uint8)
    return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)[0]   # (N,3) BGR

_TURBO_LUT = _make_turbo_lut()


def depth_at_patch(bgr_img: np.ndarray, u: int, v: int, r: int = 7):
    """
    Sample depth (metres, std) from a turbo-encoded BGR depth image
    at pixel (u,v) using a patch of radius r.
    Returns (depth_m, std_m) or (None, 999).
    """
    if bgr_img is None:
        return None, 999.0
    h, w = bgr_img.shape[:2]
    patch = bgr_img[max(0,v-r):min(h,v+r+1), max(0,u-r):min(w,u+r+1)]
    if patch.size == 0:
        return None, 999.0

    flat  = patch.reshape(-1, 3).astype(np.float32)
    step  = 8
    lut   = _TURBO_LUT[::step].astype(np.float32)         # (≈500, 3)
    diff  = flat[:, None, :] - lut[None, :, :]            # (N, M, 3)
    idx   = (diff * diff).sum(2).argmin(1) * step         # (N,)
    d     = idx.astype(np.float32) / (_TURBO_N - 1) * _TURBO_MAX
    ok    = (d > 0.15) & (d < 3.90)
    if not ok.any():
        return None, 999.0
    dv = d[ok]
    return float(np.median(dv)), float(np.std(dv))


def backproject(u, v, depth_m, fx, fy, cx, cy):
    """Pixel + depth → camera-frame 3-D point."""
    return ((u - cx) / fx * depth_m,
            (v - cy) / fy * depth_m,
            float(depth_m))


# Camera tilt / height — must match nav_depth_processor.py
_CAM_TILT  = math.radians(-25.0)   # negative = downward
_CAM_H     = 0.70                  # metres above ground


def project_rover_to_image(fwd_m, left_m, fx, fy, cx, cy):
    """
    Project a point (fwd_m forward, left_m left) in rover-body frame
    onto the depth image, assuming camera tilt and height.

    Camera frame: x=right (+u), y=down (+v), z=forward (optical axis).
    Rover body:   x=forward, y=left.

    For a ground point at height 0, camera at height H, tilt T (neg = down):
      cam_x = -left_m
      cam_z =  fwd_m * cos(T) - H * sin(T)
      cam_y =  fwd_m * sin(T) + H * cos(T)   (distance below optical axis)
    where T = |tilt| for the geometric calc (camera looks downward).
    """
    T     = abs(_CAM_TILT)
    cam_x = -left_m
    cam_z =  fwd_m * math.cos(T) - _CAM_H * math.sin(T)
    cam_y =  fwd_m * math.sin(T) + _CAM_H * math.cos(T)
    if cam_z < 0.05:
        return None, None
    u = int(round(fx * cam_x / cam_z + cx))
    v = int(round(fy * cam_y / cam_z + cy))
    return u, v


# ═══════════════════════════════════════════════════════════════
#  LUCAS-KANADE TRACKER
#  Tracks a set of named points in the RGB image.
# ═══════════════════════════════════════════════════════════════

LK_PARAMS = dict(
    winSize   = (21, 21),
    maxLevel  = 3,
    criteria  = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
)
LK_MAX_ERR = 12.0    # pixel error threshold — above this tracking failed
LK_MAX_MOVE = 60.0   # max pixels a point can move per frame (sanity check)


class LKTracker:
    """
    Sparse Lucas-Kanade tracker for a small set of named 2D points.

    Each point has:
      - name: str identifier
      - px: current (u, v) in image coords (float32)
      - lost: bool — True if tracking failed
      - age: frames since last confirmed
    """

    def __init__(self):
        self._lock   = threading.Lock()
        self._prev_gray: np.ndarray | None = None
        self._points: dict = {}   # name → {'px': np.array([u,v], f32), 'lost': bool, 'age': int}

    def add(self, name: str, u: int, v: int):
        """Add or replace a tracked point."""
        with self._lock:
            self._points[name] = {
                'px':   np.array([[u, v]], dtype=np.float32),
                'lost': False,
                'age':  0,
            }

    def remove(self, name: str):
        with self._lock:
            self._points.pop(name, None)

    def clear(self):
        with self._lock:
            self._points.clear()

    def get(self, name: str):
        """Return current (u, v) int tuple, or None if lost/missing."""
        with self._lock:
            p = self._points.get(name)
            if p is None or p['lost']:
                return None
            return (int(round(p['px'][0, 0])), int(round(p['px'][0, 1])))

    def is_lost(self, name: str):
        with self._lock:
            p = self._points.get(name)
            return p is None or p['lost']

    def update(self, rgb_bgr: np.ndarray):
        """
        Call with each new RGB frame (BGR uint8).
        Updates all tracked point positions using LK optical flow.
        """
        gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)

        with self._lock:
            prev  = self._prev_gray
            names = list(self._points.keys())
            pts   = [self._points[n]['px'].copy() for n in names]

        if prev is None or not pts:
            with self._lock:
                self._prev_gray = gray
            return

        # Stack all points into one array for batch LK
        all_pts = np.concatenate(pts, axis=0).reshape(-1, 1, 2)  # (N,1,2)

        next_pts, status, err = cv2.calcOpticalFlowPyrLK(
            prev, gray, all_pts, None, **LK_PARAMS)

        with self._lock:
            self._prev_gray = gray
            for i, name in enumerate(names):
                if name not in self._points:
                    continue
                p    = self._points[name]
                ok   = (status is not None and
                        status[i, 0] == 1 and
                        err is not None and
                        float(err[i, 0]) < LK_MAX_ERR)

                if ok:
                    new_px = next_pts[i]   # shape (1,2)
                    # Sanity: reject if point moved too far
                    old_px = all_pts[i]
                    dist   = float(np.linalg.norm(new_px - old_px))
                    if dist < LK_MAX_MOVE:
                        p['px']   = new_px
                        p['lost'] = False
                        p['age']  = 0
                    else:
                        p['age'] += 1
                        if p['age'] > 5:
                            p['lost'] = True
                else:
                    p['age'] += 1
                    if p['age'] > 5:
                        p['lost'] = True


# ═══════════════════════════════════════════════════════════════
#  CLICKABLE IMAGE LABEL
# ═══════════════════════════════════════════════════════════════

class ClickableImage(QLabel):
    clicked = pyqtSignal(int, int)

    def __init__(self):
        super().__init__()
        self.setMouseTracking(False)
        self._iw = 424
        self._ih = 240

    def set_image_size(self, w, h):
        self._iw = max(w, 1)
        self._ih = max(h, 1)

    def _to_img(self, wx, wy):
        pm = self.pixmap()
        if pm is None or pm.isNull():
            return None
        lw, lh = self.width(), self.height()
        pw, ph = pm.width(), pm.height()
        if pw == 0 or ph == 0:
            return None
        scale = min(lw / pw, lh / ph)
        dw, dh = int(pw * scale), int(ph * scale)
        ox, oy = (lw - dw) // 2, (lh - dh) // 2
        if not (ox <= wx < ox + dw and oy <= wy < oy + dh):
            return None
        u = int((wx - ox) / dw * self._iw)
        v = int((wy - oy) / dh * self._ih)
        return max(0, min(self._iw-1, u)), max(0, min(self._ih-1, v))

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            px = self._to_img(e.x(), e.y())
            if px:
                self.clicked.emit(*px)


# ═══════════════════════════════════════════════════════════════
#  ROS2 BACKEND
# ═══════════════════════════════════════════════════════════════

class RosBackend(QObject):
    sig_depth    = pyqtSignal(object)   # np.ndarray BGR depth
    sig_rgb      = pyqtSignal(object)   # np.ndarray BGR rgb
    sig_path     = pyqtSignal(object)   # list of (fwd, left) rover-local metres
    sig_status   = pyqtSignal(str)
    sig_mux      = pyqtSignal(str)
    sig_obs      = pyqtSignal(object)   # list of (fwd, left) auto obstacles
    sig_progress = pyqtSignal(object)   # dict from /nav/path_progress JSON

    def __init__(self):
        super().__init__()
        self._running = True
        self._lock    = threading.Lock()

        self._depth_bgr: np.ndarray | None = None
        self._iw = 424; self._ih = 240
        self._fx = self._fy = self._cx = self._cy = None

        self._goal_pub = self._cancel_pub = self._manual_pub = None

    def run(self):
        rclpy.init()
        n = rclpy.create_node('nav_control_panel')
        be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        n.create_subscription(CompressedImage,
            '/camera/depth/stream/compressed', self._cb_depth, be)
        n.create_subscription(CompressedImage,
            '/camera/color/stream/compressed', self._cb_rgb, be)
        n.create_subscription(CameraInfo,
            '/camera/camera/color/camera_info', self._cb_info, rel)
        n.create_subscription(Path,
            '/nav/planned_path', self._cb_path, rel)
        n.create_subscription(String,
            '/nav/status',     lambda m: self.sig_status.emit(m.data), rel)
        n.create_subscription(String,
            '/nav/mux_status', lambda m: self.sig_mux.emit(m.data),    rel)
        n.create_subscription(MarkerArray,
            '/nav/obstacle_markers', self._cb_obs, rel)
        n.create_subscription(String,
            '/nav/path_progress', self._cb_progress, rel)

        self._goal_pub   = n.create_publisher(
            Float32MultiArray, '/nav/goal_camera_frame', rel)
        self._cancel_pub = n.create_publisher(Bool, '/nav/cancel', rel)
        self._manual_pub = n.create_publisher(
            Float32MultiArray, '/nav/manual_obstacles', rel)

        while self._running and rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
        n.destroy_node()
        try: rclpy.shutdown()
        except: pass

    def stop(self): self._running = False

    # ── callbacks ──────────────────────────────────────────────

    def _cb_depth(self, msg):
        arr = np.frombuffer(bytes(msg.data), np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is not None:
            with self._lock:
                self._depth_bgr = bgr
                self._ih, self._iw = bgr.shape[:2]
            self.sig_depth.emit(bgr)

    def _cb_rgb(self, msg):
        arr = np.frombuffer(bytes(msg.data), np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is not None:
            self.sig_rgb.emit(bgr)

    def _cb_info(self, msg):
        with self._lock:
            self._fx = msg.k[0]; self._fy = msg.k[4]
            self._cx = msg.k[2]; self._cy = msg.k[5]
            self._iw = msg.width; self._ih = msg.height

    def _cb_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.sig_path.emit(pts)

    def _cb_obs(self, msg: MarkerArray):
        pts = []
        for m in msg.markers:
            if m.action == 3: pts = []; continue
            pts.append((m.pose.position.x, m.pose.position.y))
        self.sig_obs.emit(pts)

    def _cb_progress(self, msg):
        import json as _json
        try:
            data = _json.loads(msg.data)
            self.sig_progress.emit(data)
        except Exception:
            pass

    # ── accessors ─────────────────────────────────────────────

    def intrinsics(self):
        with self._lock:
            return (self._fx or 213.0, self._fy or 213.0,
                    self._cx or 212.0, self._cy or 120.0,
                    self._iw, self._ih)

    def depth_at(self, u, v, r=7):
        with self._lock:
            d = self._depth_bgr
        return depth_at_patch(d, u, v, r)

    # ── publishers ────────────────────────────────────────────

    def pub_goal(self, cx, cy, cz):
        if self._goal_pub:
            m = Float32MultiArray()
            m.data = [float(cx), float(cy), float(cz)]
            self._goal_pub.publish(m)

    def pub_manual(self, pts_3d):
        if self._manual_pub:
            m = Float32MultiArray()
            m.data = [float(v) for p in pts_3d for v in p]
            self._manual_pub.publish(m)

    def pub_cancel(self):
        if self._cancel_pub:
            m = Bool(); m.data = True
            self._cancel_pub.publish(m)


# ═══════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ═══════════════════════════════════════════════════════════════

class NavPanel(QMainWindow):

    def __init__(self, ros: RosBackend):
        super().__init__()
        self.ros    = ros
        self.lk     = LKTracker()

        self._mode = 'GOAL'   # 'GOAL' | 'OBSTACLE'

        # Tracked points info: name → {'depth': float, 'std': float, 'lost': bool}
        # The pixel position lives in self.lk; we just store depth here.
        self._goal_info: dict | None  = None    # {'depth':, 'std':}
        self._obs_info:  list         = []      # [{'name':, 'depth':, 'std':}, ...]
        self._obs_counter             = 0

        # From ROS
        self._depth_bgr: np.ndarray | None = None
        self._rgb_bgr:   np.ndarray | None = None
        self._path_pts:  list = []
        self._auto_obs:  list = []   # (fwd, left) rover-local
        self._progress:  dict = {}   # latest path_progress payload

        self._build_ui()
        self.ros.sig_depth.connect(self._on_depth)
        self.ros.sig_rgb.connect(self._on_rgb)
        self.ros.sig_path.connect(self._on_path)
        self.ros.sig_status.connect(self._on_status)
        self.ros.sig_mux.connect(self._on_mux)
        self.ros.sig_obs.connect(self._on_auto_obs)
        self.ros.sig_progress.connect(self._on_progress)

        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(120)   # ~8 Hz

    # ── UI ────────────────────────────────────────────────────

    def _build_ui(self):
        self.setWindowTitle('Rover Nav Control')
        self.setMinimumSize(1300, 660)
        self.setStyleSheet("""
            QMainWindow,QWidget{background:#0a0c10;color:#c8d8e8;
                font-family:'Courier New',monospace;font-size:11px}
            QGroupBox{border:1px solid #1a2c3c;border-radius:3px;margin-top:8px;
                padding:4px;color:#386070;font-size:10px}
            QGroupBox::title{left:8px;padding:0 3px}
            QPushButton{background:#101822;border:1px solid #1c3248;border-radius:3px;
                color:#68a4c0;padding:5px 12px;font-family:'Courier New',monospace;
                font-size:11px;min-height:26px}
            QPushButton:hover{background:#182030;border-color:#2c5070}
            QPushButton#goal_on{background:#081608;border:2px solid #18a848;
                color:#28d860;font-weight:bold}
            QPushButton#obs_on{background:#180808;border:2px solid #a82828;
                color:#d84040;font-weight:bold}
            QPushButton#danger{background:#160608;border-color:#481418;color:#a03838}
            QPushButton#danger:hover{background:#200808;border-color:#801818}
            QLabel#hdr{color:#50a0c8;font-size:13px;font-weight:bold;letter-spacing:2px}
            QLabel#dim{color:#243848;font-size:10px}
            QLabel#val{color:#50c0f0;font-size:12px}
            QLabel#ok{color:#20c060} QLabel#warn{color:#c09820} QLabel#err{color:#c02820}
            QLabel#idle{color:#283848} QLabel#nav{color:#20c060;font-weight:bold}
            QLabel#done{color:#20b090;font-weight:bold} QLabel#stuck{color:#c03820;font-weight:bold}
            QFrame#sep{background:#182030}
        """)

        root = QWidget(); self.setCentralWidget(root)
        rl = QVBoxLayout(root); rl.setContentsMargins(8,6,8,6); rl.setSpacing(5)

        # title row
        tr = QHBoxLayout()
        h = QLabel('◈  ROVER NAV CONTROL'); h.setObjectName('hdr')
        tr.addWidget(h); tr.addStretch()
        self._lbl_status = QLabel('● IDLE')
        self._lbl_status.setObjectName('idle')
        self._lbl_status.setFont(QFont('Courier New', 11, QFont.Bold))
        tr.addWidget(self._lbl_status)
        self._lbl_mux = QLabel('  JOY'); self._lbl_mux.setObjectName('warn')
        tr.addWidget(self._lbl_mux)
        rl.addLayout(tr)

        mr = QHBoxLayout(); mr.setSpacing(6); rl.addLayout(mr, 1)

        # depth panel
        dg = QGroupBox('DEPTH  ·  click to set goal or mark obstacle  '
                       '·  red=auto obstacle  ·  green=planned path')
        dl = QVBoxLayout(dg); dl.setContentsMargins(3,12,3,3)
        self._depth_lbl = ClickableImage()
        self._depth_lbl.setMinimumSize(550, 320)
        self._depth_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._depth_lbl.setAlignment(Qt.AlignCenter)
        self._depth_lbl.setStyleSheet('background:#050810;border:1px solid #142030')
        self._depth_lbl.setText('waiting for depth stream…')
        self._depth_lbl.clicked.connect(self._on_click)
        dl.addWidget(self._depth_lbl, 1)

        # tracking note under depth image
        note = QLabel('  LK optical flow tracks goal/obstacles on RGB image across frames')
        note.setObjectName('dim'); dl.addWidget(note)
        mr.addWidget(dg, 5)

        # rgb panel
        rg = QGroupBox('RGB  ·  feature tracking runs here')
        rll = QVBoxLayout(rg); rll.setContentsMargins(3,12,3,3)
        self._rgb_lbl = QLabel()
        self._rgb_lbl.setMinimumSize(330,240)
        self._rgb_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._rgb_lbl.setAlignment(Qt.AlignCenter)
        self._rgb_lbl.setStyleSheet('background:#050810;border:1px solid #142030')
        self._rgb_lbl.setText('waiting for RGB…')
        rll.addWidget(self._rgb_lbl, 1)
        mr.addWidget(rg, 3)

        # controls
        cg = QGroupBox('CONTROLS')
        cl = QVBoxLayout(cg); cl.setSpacing(5); cl.setContentsMargins(6,12,6,6)
        mr.addWidget(cg, 1)

        self._btn_goal = QPushButton('⊕  SET GOAL MODE')
        self._btn_goal.setObjectName('goal_on')
        self._btn_goal.clicked.connect(self._mode_goal); cl.addWidget(self._btn_goal)
        self._btn_obs = QPushButton('⊙  ADD OBSTACLE MODE')
        self._btn_obs.clicked.connect(self._mode_obs); cl.addWidget(self._btn_obs)
        cl.addWidget(self._sep())

        g = QGridLayout(); g.setSpacing(3)
        self._i_depth  = self._irow(g, 0, 'LAST DEPTH')
        self._i_conf   = self._irow(g, 1, 'CONFIDENCE')
        self._i_track  = self._irow(g, 2, 'GOAL TRACK')
        self._i_mobs   = self._irow(g, 3, 'MANUAL OBS')
        self._i_aobs   = self._irow(g, 4, 'AUTO OBS')
        self._i_path   = self._irow(g, 5, 'PATH PTS')
        self._i_remain = self._irow(g, 6, 'REMAINING')
        self._i_action = self._irow(g, 7, 'NEXT ACTION')
        self._i_cte    = self._irow(g, 8, 'ON-TRACK')
        cl.addLayout(g)
        cl.addStretch()
        cl.addWidget(self._sep())

        self._lbl_hint = QLabel('Click depth image\nto set goal')
        self._lbl_hint.setObjectName('dim'); self._lbl_hint.setAlignment(Qt.AlignCenter)
        cl.addWidget(self._lbl_hint)

        b = QPushButton('⊘  CLEAR MANUAL OBS'); b.setObjectName('danger')
        b.clicked.connect(self._clear_obs); cl.addWidget(b)
        b2 = QPushButton('■  CANCEL NAVIGATION'); b2.setObjectName('danger')
        b2.clicked.connect(self._cancel); cl.addWidget(b2)

        self.statusBar().setStyleSheet('background:#040610;color:#243448;font-size:10px')
        self.statusBar().showMessage('Ready — click depth image to set navigation goal')

    def _sep(self):
        f = QFrame(); f.setObjectName('sep')
        f.setFrameShape(QFrame.HLine); f.setFixedHeight(1); return f

    def _irow(self, g, row, label):
        ll = QLabel(label); ll.setObjectName('dim')
        vv = QLabel('—');   vv.setObjectName('val')
        g.addWidget(ll, row, 0); g.addWidget(vv, row, 1); return vv

    def _restyle(self, w):
        w.style().unpolish(w); w.style().polish(w)

    # ── ROS signal handlers ────────────────────────────────────

    def _on_depth(self, bgr):
        self._depth_bgr = bgr
        h, w = bgr.shape[:2]
        self._depth_lbl.set_image_size(w, h)

    def _on_rgb(self, bgr):
        self._rgb_bgr = bgr
        # Feed every RGB frame into LK tracker
        self.lk.update(bgr)
        # Draw tracking dots on RGB display
        self._draw_rgb_overlay(bgr)

    def _on_path(self, pts):
        self._path_pts = pts
        self._i_path.setText(str(len(pts)))

    def _on_status(self, s):
        lut = {'IDLE':('● IDLE','idle'), 'NAVIGATING':('▶ NAVIGATING','nav'),
               'GOAL_REACHED':('✓ GOAL','done'), 'STUCK':('⚠ STUCK','stuck')}
        text, obj = lut.get(s, (f'● {s}', 'idle'))
        self._lbl_status.setText(text); self._lbl_status.setObjectName(obj)
        self._restyle(self._lbl_status)

    def _on_mux(self, s):
        if 'AUTO' in s.upper():
            self._lbl_mux.setText('  AUTO'); self._lbl_mux.setObjectName('ok')
        else:
            self._lbl_mux.setText('  JOY');  self._lbl_mux.setObjectName('warn')
        self._restyle(self._lbl_mux)

    def _on_auto_obs(self, pts):
        self._auto_obs = pts
        self._i_aobs.setText(str(len(pts)))

    def _on_progress(self, data: dict):
        self._progress = data
        prog = data.get('progress', {})
        wpts = data.get('waypoints', [])

        # Update path_pts from progress data (always in robot-local frame)
        self._path_pts = [(w['fwd'], w['left']) for w in wpts]
        self._i_path.setText(str(len(wpts)))

        # Distance remaining
        dr = prog.get('dist_remaining_m', None)
        if dr is not None:
            self._i_remain.setText(f'{dr:.2f} m')

        # Next action
        action = prog.get('action', {})
        direction = action.get('direction', '')
        turn_deg  = action.get('turn_deg', 0)
        seg_dist  = action.get('seg_dist_m', 0)
        if direction:
            if direction == 'DRIVE STRAIGHT':
                self._i_action.setText(f'STRAIGHT {seg_dist:.1f}m')
            else:
                self._i_action.setText(f'{direction} {abs(turn_deg):.0f}° → {seg_dist:.1f}m')

        # Cross-track error (on-track indicator)
        cte = prog.get('cross_track_err_m', 0)
        on_track = prog.get('on_track', True)
        cte_text = f'±{cte:.2f}m'
        if on_track:
            self._i_cte.setText(f'YES  {cte_text}')
            self._i_cte.setObjectName('ok')
        else:
            self._i_cte.setText(f'OFF  {cte_text}')
            self._i_cte.setObjectName('err')
        self._restyle(self._i_cte)

    # ── Mode buttons ──────────────────────────────────────────

    def _mode_goal(self):
        self._mode = 'GOAL'
        self._btn_goal.setObjectName('goal_on');  self._restyle(self._btn_goal)
        self._btn_obs.setObjectName('');          self._restyle(self._btn_obs)
        self._lbl_hint.setText('Click depth image\nto set goal')
        self._depth_lbl.setCursor(Qt.CrossCursor)

    def _mode_obs(self):
        self._mode = 'OBSTACLE'
        self._btn_obs.setObjectName('obs_on');    self._restyle(self._btn_obs)
        self._btn_goal.setObjectName('');          self._restyle(self._btn_goal)
        self._lbl_hint.setText('Click depth image\nto mark a rock')
        self._depth_lbl.setCursor(Qt.PointingHandCursor)

    def _clear_obs(self):
        for info in self._obs_info:
            self.lk.remove(info['name'])
        self._obs_info.clear()
        self._i_mobs.setText('0')
        self.ros.pub_manual([])
        self.statusBar().showMessage('Manual obstacles cleared')

    def _cancel(self):
        self.ros.pub_cancel()
        self._path_pts.clear()
        self.lk.remove('goal')
        self._goal_info = None
        self.statusBar().showMessage('Navigation cancelled')

    # ── Click handler ─────────────────────────────────────────

    def _on_click(self, u: int, v: int):
        """
        User clicked pixel (u,v) on the DEPTH image.
        1. Read depth there.
        2. Register the corresponding RGB pixel with the LK tracker.
        3. Send initial 3D goal/obstacle to miniPC.
        """
        depth, std = self.ros.depth_at(u, v)

        if depth is None:
            self.statusBar().showMessage(
                f'⚠  No depth at ({u},{v}) — click a closer/brighter area')
            return

        self._i_depth.setText(f'{depth:.2f} m')
        if   std < 0.08: conf, cobj = f'HIGH ±{std*100:.0f}cm', 'ok'
        elif std < 0.20: conf, cobj = f'MED  ±{std*100:.0f}cm', 'warn'
        else:            conf, cobj = f'LOW  ±{std*100:.0f}cm', 'err'
        self._i_conf.setText(conf); self._i_conf.setObjectName(cobj)
        self._restyle(self._i_conf)

        fx, fy, cx, cy, iw, ih = self.ros.intrinsics()

        if self._mode == 'GOAL':
            # Register point with LK tracker on the RGB image at (u,v)
            self.lk.add('goal', u, v)
            self._goal_info = {'depth': depth, 'std': std}
            self._i_track.setText('TRACKING')
            self._i_track.setObjectName('ok'); self._restyle(self._i_track)

            # Send immediately
            self.ros.pub_goal(*backproject(u, v, depth, fx, fy, cx, cy))
            self.statusBar().showMessage(
                f'Goal set  px=({u},{v})  depth={depth:.2f}m  '
                f'conf={conf}  →  LK tracking active')

        elif self._mode == 'OBSTACLE':
            name = f'obs_{self._obs_counter}'
            self._obs_counter += 1
            self.lk.add(name, u, v)
            self._obs_info.append({'name': name, 'depth': depth, 'std': std})
            self._i_mobs.setText(str(len(self._obs_info)))

            pts_3d = self._collect_manual_3d(fx, fy, cx, cy)
            self.ros.pub_manual(pts_3d)
            self.statusBar().showMessage(
                f'Manual obstacle #{len(self._obs_info)} '
                f'at ({u},{v}) depth={depth:.2f}m — LK tracking active')

    # ── Periodic tick ─────────────────────────────────────────

    def _tick(self):
        """
        ~8 Hz.  Read tracked pixel positions from LK tracker,
        sample fresh depth, re-send goal and obstacles to miniPC,
        redraw depth overlay.
        """
        if self._depth_bgr is None:
            return

        fx, fy, cx, cy, iw, ih = self.ros.intrinsics()

        # Goal: get current tracked pixel, sample depth, send
        if self._goal_info is not None:
            px = self.lk.get('goal')
            if px is not None:
                u, v = px
                d, _ = self.ros.depth_at(u, v, r=5)
                if d is not None and d > 0.15:
                    self._goal_info['depth'] = d
                    self.ros.pub_goal(*backproject(u, v, d, fx, fy, cx, cy))
                self._i_track.setText('TRACKING')
                self._i_track.setObjectName('ok')
            else:
                self._i_track.setText('LOST ← re-click')
                self._i_track.setObjectName('err')
            self._restyle(self._i_track)

        # Manual obstacles: get tracked pixels, send
        if self._obs_info:
            pts_3d = self._collect_manual_3d(fx, fy, cx, cy)
            if pts_3d:
                self.ros.pub_manual(pts_3d)

        # Redraw depth image with overlays
        self._redraw(fx, fy, cx, cy, iw, ih)

    def _collect_manual_3d(self, fx, fy, cx, cy):
        """Collect current 3D positions of all tracked manual obstacles."""
        pts_3d = []
        for info in self._obs_info:
            px = self.lk.get(info['name'])
            if px is None:
                continue
            u, v = px
            d, _ = self.ros.depth_at(u, v, r=4)
            if d is None:
                d = info['depth']   # use last known
            else:
                info['depth'] = d
            pts_3d.append(backproject(u, v, d, fx, fy, cx, cy))
        return pts_3d

    # ── Draw overlays on depth image ──────────────────────────

    def _redraw(self, fx, fy, cx, cy, iw, ih):
        frame = self._depth_bgr.copy()

        # ── Auto-detected obstacles ────────────────────────────────────────
        for (fwd, left) in self._auto_obs:
            pu, pv = project_rover_to_image(fwd, left, fx, fy, cx, cy)
            if pu is None: continue
            if not (0 <= pu < iw and 0 <= pv < ih): continue
            cell = max(4, int(fx * 0.15 / max(fwd, 0.3)))
            cv2.rectangle(frame,
                (pu-cell, pv-cell), (pu+cell, pv+cell), (0, 30, 160), -1)
            cv2.rectangle(frame,
                (pu-cell, pv-cell), (pu+cell, pv+cell), (0, 60, 240), 1)

        # ── Planned path with progress annotations ─────────────────────────
        wpts    = self._progress.get('waypoints', [])
        prog    = self._progress.get('progress', {})
        cur_idx = prog.get('waypoint_idx', 0)
        action  = prog.get('action', {})
        on_trk  = prog.get('on_track', True)

        # Build projected pixel list from waypoints (already robot-local)
        path_px = []   # list of (pu, pv, waypoint_dict)
        for w in wpts:
            pu, pv = project_rover_to_image(w['fwd'], w['left'], fx, fy, cx, cy)
            if pu is None:
                path_px.append(None)
            else:
                path_px.append((pu, pv, w))

        # Draw path in two passes: done segment (grey) then remaining (green/amber)
        # Pass 1: segments already traversed — dim grey
        prev_px = None
        for i, p in enumerate(path_px):
            if p is None: prev_px = None; continue
            pu, pv, w = p
            if prev_px and w['done']:
                cv2.line(frame, prev_px, (pu, pv), (50, 60, 50), 1, cv2.LINE_AA)
            prev_px = (pu, pv) if w['done'] else None

        # Pass 2: remaining path — bright green, thicker
        valid_remaining = [(p[0], p[1]) for p in path_px
                           if p is not None and not p[2]['done']]
        if len(valid_remaining) >= 2:
            col_line  = (20, 200, 55) if on_trk else (20, 160, 220)  # green or amber
            col_glow  = (0,  60,  15) if on_trk else (0,   50, 70)
            arr = np.array(valid_remaining, np.int32).reshape(-1,1,2)
            cv2.polylines(frame, [arr], False, col_glow, 7, cv2.LINE_AA)   # glow
            cv2.polylines(frame, [arr], False, col_line, 2, cv2.LINE_AA)   # line

        # Pass 3: distance markers every 0.5 m along the remaining path
        # We accumulate cum_dist from the current waypoint
        cum_target   = 0.5
        prev_cum     = 0.0
        prev_pt      = None
        marker_label = 0.5

        for p in path_px:
            if p is None: prev_pt = None; continue
            pu, pv, w = p
            if w['done']: prev_pt = (pu, pv, w['cum_m']); continue

            if prev_pt is not None:
                seg = w['cum_m'] - prev_pt[2]
                while prev_cum + seg >= cum_target:
                    # Interpolate position at cum_target
                    frac = (cum_target - prev_cum) / max(seg, 1e-6)
                    mx   = int(prev_pt[0] + frac * (pu - prev_pt[0]))
                    my   = int(prev_pt[1] + frac * (pv - prev_pt[1]))
                    if 0 <= mx < iw and 0 <= my < ih:
                        # Tick mark
                        cv2.circle(frame, (mx, my), 4, (200, 220, 50), -1)
                        cv2.circle(frame, (mx, my), 4, (255, 255, 255), 1)
                        # Distance label
                        lbl = f'{marker_label:.1f}m'
                        cv2.putText(frame, lbl, (mx+6, my-4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.34,
                                    (180, 200, 40), 1, cv2.LINE_AA)
                    cum_target   += 0.5
                    marker_label += 0.5
                prev_cum += seg

            prev_pt = (pu, pv, w['cum_m'])

        # Pass 4: goal endpoint
        if valid_remaining:
            ep = valid_remaining[-1]
            cv2.circle(frame, ep, 9, (0, 190, 255), -1)
            cv2.circle(frame, ep, 9, (255, 255, 255), 1)
            dr = prog.get('dist_remaining_m')
            if dr is not None:
                cv2.putText(frame, f'{dr:.1f}m', (ep[0]+11, ep[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                            (0, 200, 255), 1, cv2.LINE_AA)

        # Pass 5: current waypoint marker + immediate action arrow
        if cur_idx < len(path_px) and path_px[cur_idx] is not None:
            pu, pv, _ = path_px[cur_idx]
            if 0 <= pu < iw and 0 <= pv < ih:
                cv2.circle(frame, (pu, pv), 6, (255, 220, 0), -1)

                # Draw turn arrow from rover (bottom-centre) toward next waypoint
                rover_px = (iw // 2, ih - 10)
                direction = action.get('direction', '')
                turn_deg  = action.get('turn_deg', 0)
                seg_dist  = action.get('seg_dist_m', 0)

                if direction:
                    # Arrow from rover pixel toward current target
                    cv2.arrowedLine(frame, rover_px, (pu, pv),
                                    (255, 220, 0), 2, cv2.LINE_AA, tipLength=0.25)
                    # Action label near rover
                    if direction == 'DRIVE STRAIGHT':
                        act_lbl = f'▲ {seg_dist:.1f}m'
                    else:
                        arrow_sym = '◄' if turn_deg > 0 else '►'
                        act_lbl   = f'{arrow_sym} {abs(turn_deg):.0f}° {seg_dist:.1f}m'
                    cv2.putText(frame, act_lbl, (rover_px[0]-40, rover_px[1]-16),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                                (255, 220, 0), 1, cv2.LINE_AA)

        # ── Manual obstacles at tracked pixels ────────────────────────────
        for i, info in enumerate(self._obs_info):
            px = self.lk.get(info['name'])
            lost = self.lk.is_lost(info['name'])
            if px is None: continue
            u, v = px
            col = (30, 30, 180) if not lost else (60, 60, 80)
            cv2.circle(frame, (u, v), 14, col, 2)
            cv2.drawMarker(frame, (u, v), col,
                           cv2.MARKER_TILTED_CROSS, 18, 2, cv2.LINE_AA)
            label = f'M{i+1}' + (' ??' if lost else f' {info["depth"]:.1f}m')
            cv2.putText(frame, label, (u+16, v-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)

        # ── Goal at tracked pixel ──────────────────────────────────────────
        if self._goal_info is not None:
            px   = self.lk.get('goal')
            lost = self.lk.is_lost('goal')
            d    = self._goal_info['depth']
            std  = self._goal_info['std']
            if px is not None:
                u, v = px
                ring = (30, 200, 30) if not lost and std < 0.08 else \
                       (20, 160, 200) if not lost and std < 0.20 else \
                       (60, 60, 60)
                cv2.circle(frame, (u, v), 19, (0,0,0), 3)
                cv2.circle(frame, (u, v), 18, ring, 2)
                cv2.drawMarker(frame, (u, v), ring,
                               cv2.MARKER_CROSS, 26, 2, cv2.LINE_AA)
                status_lbl = 'LOST-re-click' if lost else f'{d:.2f}m'
                cv2.putText(frame, status_lbl, (u+21, v-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, ring, 1, cv2.LINE_AA)
                cv2.putText(frame, 'GOAL', (u+21, v+12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, ring, 1, cv2.LINE_AA)

        # ── On-track / off-track banner ────────────────────────────────────
        cte = prog.get('cross_track_err_m', 0)
        if prog and not on_trk:
            cv2.rectangle(frame, (0, ih-22), (iw, ih), (0, 0, 40), -1)
            cv2.putText(frame, f'OFF TRACK  ±{cte:.2f}m — CORRECTING',
                        (5, ih-7), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                        (50, 100, 255), 1, cv2.LINE_AA)

        # ── Mode banner ────────────────────────────────────────────────────
        if self._mode == 'OBSTACLE':
            cv2.rectangle(frame, (0, 0), (iw, 20), (25, 0, 0), -1)
            cv2.putText(frame, '[ OBSTACLE MODE — click rocks to mark ]',
                        (5, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                        (70, 50, 200), 1, cv2.LINE_AA)

        # ── HUD top-right ──────────────────────────────────────────────────
        n_auto = len(self._auto_obs)
        n_man  = len(self._obs_info)
        hud    = f'AUTO:{n_auto}  MANUAL:{n_man}'
        cv2.putText(frame, hud, (5, ih-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (30, 60, 100), 1, cv2.LINE_AA)

        self._show(self._depth_lbl, frame)

    # ── Draw tracking dots on RGB ──────────────────────────────

    def _draw_rgb_overlay(self, bgr: np.ndarray):
        frame = bgr.copy()

        # Goal tracking dot
        px = self.lk.get('goal')
        if px:
            lost = self.lk.is_lost('goal')
            col  = (20, 200, 20) if not lost else (80, 80, 80)
            cv2.circle(frame, px, 8, col, 2)
            cv2.putText(frame, 'GOAL', (px[0]+10, px[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1, cv2.LINE_AA)

        # Obstacle tracking dots
        for i, info in enumerate(self._obs_info):
            px2 = self.lk.get(info['name'])
            if px2:
                lost = self.lk.is_lost(info['name'])
                col  = (20, 50, 220) if not lost else (60, 60, 60)
                cv2.circle(frame, px2, 7, col, 2)
                cv2.putText(frame, f'M{i+1}', (px2[0]+9, px2[1]-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1, cv2.LINE_AA)

        self._show(self._rgb_lbl, frame)

    # ── Utility ───────────────────────────────────────────────

    def _show(self, label: QLabel, bgr: np.ndarray):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, c = rgb.shape
        qi = QImage(rgb.data.tobytes(), w, h, c*w, QImage.Format_RGB888)
        pm = QPixmap.fromImage(qi).scaled(
            label.width(), label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(pm)

    def closeEvent(self, e):
        self._timer.stop(); self.ros.stop(); e.accept()


# ═══════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setApplicationName('Rover Nav Control')

    ros    = RosBackend()
    thread = QThread()
    ros.moveToThread(thread)
    thread.started.connect(ros.run)
    thread.start()

    win = NavPanel(ros)
    win.show()
    ret = app.exec_()
    ros.stop()
    thread.quit()
    thread.wait(3000)
    sys.exit(ret)


if __name__ == '__main__':
    main()