#!/usr/bin/env python3
"""
nav_click_laptop.py  —  LAPTOP
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Shows depth + RGB images.  Click either → rover drives there.

GOAL MARKER stays locked to the physical feature you clicked:
  • Finds the strongest corner feature within 30px of your click
  • Tracks it with Lucas-Kanade across every RGB frame
  • If tracking drops quality, shows warning but keeps last position

DISTANCE REMAINING comes from /nav/dist_remaining, which the
miniPC publishes based on IMU-measured actual movement only.
"""

import sys, threading
import numpy as np
import cv2

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg    import Float32, Bool, String

from PyQt5.QtWidgets  import (QApplication, QMainWindow, QWidget, QLabel,
                               QPushButton, QHBoxLayout, QVBoxLayout,
                               QGroupBox, QSizePolicy)
from PyQt5.QtCore     import Qt, pyqtSignal, QObject, QThread
from PyQt5.QtGui      import QImage, QPixmap

# ── Depth decode (turbo colourmap → metres) ───────────────────────────────────

_LUT_N, _LUT_MAX = 4001, 4.0

def _build_lut():
    i = np.arange(_LUT_N, dtype=np.float32).reshape(1,_LUT_N,1)
    u = (i/(_LUT_N-1)*255).astype(np.uint8)
    return cv2.applyColorMap(u, cv2.COLORMAP_TURBO)[0].astype(np.float32)

_TURBO = _build_lut()

def depth_at_pixel(bgr, u, v, r=10):
    if bgr is None: return None
    h,w = bgr.shape[:2]
    patch = bgr[max(0,v-r):min(h,v+r+1), max(0,u-r):min(w,u+r+1)]
    if patch.size == 0: return None
    flat = patch.reshape(-1,3).astype(np.float32)
    lut  = _TURBO[::8]
    idx  = ((flat[:,None,:]-lut[None,:,:])**2).sum(2).argmin(1)*8
    d    = idx.astype(np.float32)/(_LUT_N-1)*_LUT_MAX
    ok   = (d>0.15)&(d<3.90)
    return float(np.median(d[ok])) if ok.any() else None

# ── Feature tracker ───────────────────────────────────────────────────────────

LK = dict(winSize=(21,21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT, 30, 0.01))

class Tracker:
    """
    Tracks a patch of features near the clicked point.
    Uses goodFeaturesToTrack to find the best corner near the click,
    then LK optical flow to follow it.
    Falls back to centroid of patch if no corner found.
    """
    def __init__(self):
        self._pt        = None   # np array shape (1,1,2) float32
        self._prev_gray = None
        self._ok        = False
        self._lock      = threading.Lock()

    def set_point(self, u: int, v: int, gray: np.ndarray):
        h, w = gray.shape
        r = 30
        roi = gray[max(0,v-r):min(h,v+r), max(0,u-r):min(w,u+r)]

        # Find best feature in patch
        corners = None
        if roi.size > 0:
            corners = cv2.goodFeaturesToTrack(
                roi, maxCorners=1, qualityLevel=0.01,
                minDistance=5, blockSize=7)

        if corners is not None and len(corners) > 0:
            cx = corners[0,0,0] + max(0, u-r)
            cy = corners[0,0,1] + max(0, v-r)
            pt = np.array([[[cx, cy]]], dtype=np.float32)
            print(f'[TRACK] found corner at ({cx:.0f},{cy:.0f}), '
                  f'click was ({u},{v})', flush=True)
        else:
            # No corner — track the centre pixel directly
            pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
            print(f'[TRACK] no corner found, tracking centre pixel ({u},{v})',
                  flush=True)

        with self._lock:
            self._pt        = pt
            self._prev_gray = gray.copy()
            self._ok        = True

    def update(self, gray: np.ndarray):
        """Returns (u, v, ok) — call on every new frame."""
        with self._lock:
            if not self._ok or self._pt is None or self._prev_gray is None:
                self._prev_gray = gray.copy()
                return None, None, False

            new_pt, status, err = cv2.calcOpticalFlowPyrLK(
                self._prev_gray, gray, self._pt, None, **LK)

            tracked = (status is not None and status[0,0,0] == 1
                       and err is not None and err[0,0,0] < 20.0)

            if tracked:
                # Sanity check: didn't jump more than 80px
                dx = new_pt[0,0,0] - self._pt[0,0,0]
                dy = new_pt[0,0,1] - self._pt[0,0,1]
                if dx*dx + dy*dy > 80**2:
                    tracked = False

            if tracked:
                self._pt        = new_pt
                self._prev_gray = gray.copy()
                return int(round(new_pt[0,0,0])), int(round(new_pt[0,0,1])), True
            else:
                # Keep previous position — don't lose it on a single bad frame
                self._prev_gray = gray.copy()
                u = int(round(self._pt[0,0,0]))
                v = int(round(self._pt[0,0,1]))
                return u, v, False   # position known but quality dropped

    def reset(self):
        with self._lock:
            self._pt   = None
            self._ok   = False

# ── Clickable label ───────────────────────────────────────────────────────────

class ClickLabel(QLabel):
    clicked_pixel = pyqtSignal(int,int)
    def __init__(self):
        super().__init__()
        self._iw=424; self._ih=240
    def set_image_size(self,w,h):
        self._iw=max(1,w); self._ih=max(1,h)
    def mousePressEvent(self,ev):
        if ev.button()!=Qt.LeftButton: return
        pm=self.pixmap()
        if pm is None or pm.isNull(): return
        lw,lh=self.width(),self.height()
        pw,ph=pm.width(),pm.height()
        if pw==0 or ph==0: return
        scale=min(lw/pw,lh/ph)
        dw,dh=int(pw*scale),int(ph*scale)
        ox,oy=(lw-dw)//2,(lh-dh)//2
        wx,wy=ev.x(),ev.y()
        if not(ox<=wx<ox+dw and oy<=wy<oy+dh): return
        u=int((wx-ox)/dw*self._iw); v=int((wy-oy)/dh*self._ih)
        self.clicked_pixel.emit(max(0,min(self._iw-1,u)),
                                max(0,min(self._ih-1,v)))

# ── ROS backend ───────────────────────────────────────────────────────────────

class RosBackend(QObject):
    sig_depth     = pyqtSignal(object)
    sig_rgb       = pyqtSignal(object)
    sig_status    = pyqtSignal(str)
    sig_remaining = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self._lock      = threading.Lock()
        self._depth_bgr = None
        self._running   = True
        self._goal_pub = self._cancel_pub = None

    def run(self):
        rclpy.init()
        node = rclpy.create_node('nav_click_laptop')
        be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=2)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        def _cb_depth(msg):
            arr=np.frombuffer(bytes(msg.data),np.uint8)
            bgr=cv2.imdecode(arr,cv2.IMREAD_COLOR)
            if bgr is not None:
                with self._lock: self._depth_bgr=bgr
                self.sig_depth.emit(bgr)

        def _cb_rgb(msg):
            arr=np.frombuffer(bytes(msg.data),np.uint8)
            bgr=cv2.imdecode(arr,cv2.IMREAD_COLOR)
            if bgr is not None: self.sig_rgb.emit(bgr)

        node.create_subscription(CompressedImage,
            '/camera/depth/stream/compressed',_cb_depth,be)
        node.create_subscription(CompressedImage,
            '/camera/color/stream/compressed',_cb_rgb,be)
        node.create_subscription(String,'/nav/status',
            lambda m: self.sig_status.emit(m.data),rel)
        node.create_subscription(Float32,'/nav/dist_remaining',
            lambda m: self.sig_remaining.emit(float(m.data)),rel)

        self._goal_pub   = node.create_publisher(Float32,'/nav/goal_dist',rel)
        self._cancel_pub = node.create_publisher(Bool,   '/nav/cancel',   rel)

        while self._running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

    def stop(self): self._running=False
    def get_depth(self):
        with self._lock:
            return self._depth_bgr.copy() if self._depth_bgr is not None else None
    def send_goal(self,m):
        if self._goal_pub:
            msg=Float32(); msg.data=float(m); self._goal_pub.publish(msg)
    def send_cancel(self):
        if self._cancel_pub:
            msg=Bool(); msg.data=True; self._cancel_pub.publish(msg)

# ── Main window ───────────────────────────────────────────────────────────────

STYLE="""
QMainWindow,QWidget{background:#090c12;color:#c0d0e0;
    font-family:'Courier New',monospace;font-size:12px}
QGroupBox{border:1px solid #1a2838;border-radius:4px;margin-top:10px;
    padding:4px;color:#305878;font-size:11px}
QPushButton{background:#0e1620;border:1px solid #1c3040;border-radius:4px;
    color:#50a0c0;padding:6px 16px;min-height:28px}
QPushButton:hover{background:#162030}
QPushButton#btnCancel{background:#160a0a;border-color:#401020;color:#c04060}
"""

class MainWindow(QMainWindow):
    def __init__(self, ros: RosBackend):
        super().__init__()
        self._ros       = ros
        self._depth_bgr = None
        self._tracker   = Tracker()
        self._goal_dist = None
        self._goal_uv   = None     # last known tracked position
        self._track_ok  = False    # True = LK quality good this frame
        self._has_goal  = False

        self.setWindowTitle('Rover Nav — click to drive')
        self.setMinimumSize(1000,480)
        self.setStyleSheet(STYLE)
        self._build_ui()

        ros.sig_depth.connect(self._on_depth)
        ros.sig_rgb.connect(self._on_rgb)
        ros.sig_status.connect(self._on_status)
        ros.sig_remaining.connect(self._on_remaining)

    def _build_ui(self):
        root=QWidget(); self.setCentralWidget(root)
        hl=QHBoxLayout(root); hl.setContentsMargins(8,8,8,8); hl.setSpacing(8)

        dg=QGroupBox('DEPTH — click to set goal')
        dl=QVBoxLayout(dg); dl.setContentsMargins(4,14,4,4)
        self._depth_lbl=ClickLabel()
        self._depth_lbl.setMinimumSize(480,300)
        self._depth_lbl.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self._depth_lbl.setAlignment(Qt.AlignCenter)
        self._depth_lbl.setStyleSheet('background:#04080e')
        self._depth_lbl.setText('⏳ waiting for depth…')
        self._depth_lbl.clicked_pixel.connect(self._on_click)
        dl.addWidget(self._depth_lbl); hl.addWidget(dg,5)

        rg=QGroupBox('RGB — click to set goal')
        rl=QVBoxLayout(rg); rl.setContentsMargins(4,14,4,4)
        self._rgb_lbl=ClickLabel()
        self._rgb_lbl.setMinimumSize(320,240)
        self._rgb_lbl.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self._rgb_lbl.setAlignment(Qt.AlignCenter)
        self._rgb_lbl.setStyleSheet('background:#04080e')
        self._rgb_lbl.setText('⏳ waiting for RGB…')
        self._rgb_lbl.clicked_pixel.connect(self._on_click)
        rl.addWidget(self._rgb_lbl); hl.addWidget(rg,4)

        cg=QGroupBox('CONTROLS'); cl=QVBoxLayout(cg)
        cl.setContentsMargins(10,14,10,10); cl.setSpacing(8)
        hl.addWidget(cg,2)

        cl.addWidget(QLabel('DISTANCE REMAINING'))
        self._dist_lbl=QLabel('—')
        self._dist_lbl.setAlignment(Qt.AlignCenter)
        self._dist_lbl.setStyleSheet('color:#20d060;font-size:38px;font-weight:bold')
        cl.addWidget(self._dist_lbl)

        self._goal_lbl=QLabel('no goal set')
        self._goal_lbl.setAlignment(Qt.AlignCenter)
        self._goal_lbl.setStyleSheet('color:#305878;font-size:11px')
        cl.addWidget(self._goal_lbl)

        self._track_lbl=QLabel('')
        self._track_lbl.setAlignment(Qt.AlignCenter)
        self._track_lbl.setStyleSheet('color:#c0a020;font-size:10px')
        cl.addWidget(self._track_lbl)

        self._status_lbl=QLabel('IDLE')
        self._status_lbl.setAlignment(Qt.AlignCenter)
        self._status_lbl.setStyleSheet('color:#506070;font-size:13px;font-weight:bold')
        cl.addWidget(self._status_lbl)

        cl.addStretch()
        btn=QPushButton('■  STOP'); btn.setObjectName('btnCancel')
        btn.clicked.connect(self._on_cancel); cl.addWidget(btn)
        hint=QLabel('Click either image\nto drive to that spot.\nGoal tracks the feature.')
        hint.setAlignment(Qt.AlignCenter)
        hint.setStyleSheet('color:#2a3c50;font-size:10px')
        cl.addWidget(hint)

        self.statusBar().setStyleSheet('background:#040810;color:#304050;font-size:10px')
        self.statusBar().showMessage('Click an image to set a goal')

    # ── Incoming frames ───────────────────────────────────────────────────

    def _on_depth(self, bgr):
        self._depth_bgr = bgr
        h,w=bgr.shape[:2]
        self._depth_lbl.set_image_size(w,h)
        frame=bgr.copy()
        self._draw_goal(frame)
        self._show(self._depth_lbl,frame)

    def _on_rgb(self, bgr):
        h,w=bgr.shape[:2]
        self._rgb_lbl.set_image_size(w,h)
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        frame=bgr.copy()

        if self._has_goal:
            u,v,ok=self._tracker.update(gray)
            if u is not None:
                self._goal_uv  = (u,v)
                self._track_ok = ok
                self._track_lbl.setText(
                    f'tracking ({u},{v})' if ok else f'⚠ low quality ({u},{v})')
            self._draw_goal(frame)

        self._show(self._rgb_lbl,frame)

    def _draw_goal(self, frame):
        if not self._has_goal or self._goal_uv is None:
            return
        u,v=self._goal_uv
        col=(0,255,80) if self._track_ok else (200,200,0)
        cv2.drawMarker(frame,(u,v),col,cv2.MARKER_CROSS,32,2,cv2.LINE_AA)
        cv2.circle(frame,(u,v),24,col,2,cv2.LINE_AA)
        if self._goal_dist:
            cv2.putText(frame,f'GOAL {self._goal_dist:.2f}m',
                        (u+26,v-8),cv2.FONT_HERSHEY_SIMPLEX,0.55,col,2,cv2.LINE_AA)

    # ── Click ─────────────────────────────────────────────────────────────

    def _on_click(self, u: int, v: int):
        d=depth_at_pixel(self._depth_bgr,u,v)
        if d is None:
            self.statusBar().showMessage(f'⚠ no depth at ({u},{v}) — try another spot')
            return

        # Start tracking from current greyscale frame
        if self._depth_bgr is not None:
            gray=cv2.cvtColor(self._depth_bgr,cv2.COLOR_BGR2GRAY)
            self._tracker.set_point(u,v,gray)

        self._goal_dist = d
        self._goal_uv   = (u,v)
        self._track_ok  = True
        self._has_goal  = True

        self._goal_lbl.setText(f'goal: {d:.2f}m  pixel ({u},{v})')
        self._dist_lbl.setText(f'{d:.2f}m')
        self._dist_lbl.setStyleSheet('color:#20d060;font-size:38px;font-weight:bold')
        self._ros.send_goal(d)
        self.statusBar().showMessage(f'Goal sent: {d:.2f}m  [{u},{v}]')
        print(f'[CLICK] ({u},{v}) depth={d:.2f}m', flush=True)

    def _on_cancel(self):
        self._ros.send_cancel()
        self._tracker.reset()
        self._has_goal  = False
        self._goal_uv   = None
        self._goal_dist = None
        self._dist_lbl.setText('—')
        self._dist_lbl.setStyleSheet('color:#506070;font-size:38px;font-weight:bold')
        self._track_lbl.setText('')
        self._goal_lbl.setText('no goal set')
        self.statusBar().showMessage('Cancelled')

    def _on_status(self, s):
        cols={'IDLE':'#506070','NAVIGATING':'#20d060','GOAL_REACHED':'#20b890'}
        self._status_lbl.setText(s)
        self._status_lbl.setStyleSheet(
            f'color:{cols.get(s,"#c0a020")};font-size:13px;font-weight:bold')
        if s=='GOAL_REACHED':
            self._dist_lbl.setText('ARRIVED')
            self._dist_lbl.setStyleSheet('color:#20b890;font-size:32px;font-weight:bold')

    def _on_remaining(self, m: float):
        if m>0.01:
            self._dist_lbl.setText(f'{m:.2f}m')
            self._dist_lbl.setStyleSheet('color:#20d060;font-size:38px;font-weight:bold')

    def _show(self, label, bgr):
        rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        h,w,c=rgb.shape
        qi=QImage(rgb.data.tobytes(),w,h,c*w,QImage.Format_RGB888)
        pm=QPixmap.fromImage(qi).scaled(
            label.width(),label.height(),
            Qt.KeepAspectRatio,Qt.SmoothTransformation)
        label.setPixmap(pm)

    def closeEvent(self, ev):
        self._ros.stop(); ev.accept()


def main():
    app=QApplication(sys.argv)
    ros=RosBackend()
    thread=QThread()
    ros.moveToThread(thread)
    thread.started.connect(ros.run)
    thread.start()
    win=MainWindow(ros); win.show()
    ret=app.exec_()
    ros.stop(); thread.quit(); thread.wait(3000)
    sys.exit(ret)

if __name__=='__main__':
    main()