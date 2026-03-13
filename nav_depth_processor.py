#!/usr/bin/env python3
"""
nav_depth_processor.py  —  MINI PC  (v4)

Changes from v3:
  1. DRIVING BACKWARDS
     positive linear.x was driving the rover in reverse.
     REVERSE_LINEAR = True negates linear.x before publishing.
     Toggle to False if it still goes backward.

  2. GOAL NEVER STOPS
     Camera stop criterion was unreliable (lower-patch depth doesn't track
     click depth well).  Now the PRIMARY stop is the odometer (driven_m >=
     click_depth - margin).  Camera stop is removed as a primary criterion
     and replaced with a simple "depth ahead is very small" emergency stop.
     DR fallback kept but with generous tolerance.

  3. OSCILLATING TURNS
     Pure proportional angular gave constant left-right hunting.
     Fixed with:
       a) Heading DEADBAND: if error < HEADING_DEADBAND_DEG, zero angular output
       b) Derivative damping: subtract K_D * (yaw_err - prev_yaw_err) from angular
       c) Angular output only changes if error changed significantly (hysteresis)

  4. STRAIGHT-LINE TOLERANCE
     If the lookahead target is within the centre third of the camera frame
     AND closer than CENTRE_ZONE_M, suppress all angular output.
     The "centre third" maps to roughly ±(FOV/6) = ±14.5° from straight ahead.
"""

import math
import time
import threading
from heapq import heappush, heappop

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point, TransformStamped
from std_msgs.msg import Bool, String, Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

try:
    from tf2_ros import TransformBroadcaster
    _HAS_TF2 = True
except ImportError:
    _HAS_TF2 = False


# ══════════════════════════════════════════════════════════════════════════════
#  HARDWARE — adjust these two flags to match your wiring
# ══════════════════════════════════════════════════════════════════════════════

# Negate angular.z to compensate for right-side motors being wired in reverse.
# arduino_motor_controller: left=lin-ang, right=lin+ang but right is flipped.
RIGHT_FLIP_COMPENSATION = True

# Negate linear.x because positive cmd_vel linear drives the rover backward.
REVERSE_LINEAR = True


# ══════════════════════════════════════════════════════════════════════════════
#  CAMERA
# ══════════════════════════════════════════════════════════════════════════════
CAM_HEIGHT_M = 0.75
CAM_TILT_DEG = -25.0
CAM_TILT_RAD = math.radians(CAM_TILT_DEG)
CAM_HFOV_DEG = 87.0   # horizontal field of view


# ══════════════════════════════════════════════════════════════════════════════
#  OBSTACLE GRID
# ══════════════════════════════════════════════════════════════════════════════
ROCK_HEIGHT_MIN_M = 0.10
ROCK_HEIGHT_MAX_M = 0.55
BLIND_ZONE_M      = 0.70
GRID_RES_M        = 0.05
GRID_RANGE_M      = 4.0
GRID_CELLS        = int(2 * GRID_RANGE_M / GRID_RES_M)   # 160
SAFETY_RADIUS_M   = 0.80


# ══════════════════════════════════════════════════════════════════════════════
#  NAVIGATION TUNING
# ══════════════════════════════════════════════════════════════════════════════

# Speed
MAX_LINEAR  = 0.28   # m/s
MAX_ANGULAR = 0.40   # rad/s
MIN_CREEP   = 0.06   # m/s — minimum forward speed during turns

# Pure pursuit
LOOKAHEAD_M    = 1.40
WAYPOINT_TOL_M = 0.25

# ── Steering (PD controller with deadband) ────────────────────────────────────
# Proportional gain: angular = K_P * yaw_err (rad), clipped to MAX_ANGULAR
K_P = 0.55   # rad/s per radian of error — tune down if still oscillating

# Derivative gain: damps overshoot.  angular -= K_D * d(yaw_err)/dt
# Set to 0 to disable.
K_D = 0.08   # rad/s per (rad/s of error rate)

# Deadband: if |yaw_err| < this, output zero angular (no micro-corrections)
HEADING_DEADBAND_DEG = 8.0

# Centre-zone suppression: if target is straight-ish AND close, zero angular.
# "Straight-ish" = within ±(CAM_HFOV/6) of centre = ±14.5° for 87° FOV
CENTRE_ZONE_DEG  = CAM_HFOV_DEG / 6.0   # ~14.5°
CENTRE_ZONE_M    = 2.5    # suppress turns when target < this distance AND in zone

# Speed reduction during turns
HEADING_SLOW_DEG = 20.0   # start reducing speed
HEADING_STOP_DEG = 50.0   # apply floor speed
TURN_FWD_FLOOR   = 0.20   # fraction of MAX_LINEAR kept during large turns

# ── Goal stop ─────────────────────────────────────────────────────────────────
# Primary: odometer driven_m >= (click_depth - GOAL_ODOMETER_MARGIN)
GOAL_ODOMETER_MARGIN  = 0.40   # m — stop this far before the clicked point
# Emergency: camera depth ahead is very close (obstacle / arrived)
GOAL_CAM_EMERGENCY_M  = 0.55   # m — stop if depth ahead drops below this
# Fallback: DR world distance
GOAL_DR_TOL_M         = 0.60   # m

# ── Replanning ────────────────────────────────────────────────────────────────
REPLAN_HZ  = 1.5
CONTROL_HZ = 10.0

# ── Stuck ─────────────────────────────────────────────────────────────────────
STUCK_TIME_S = 5.0
STUCK_DIST_M = 0.10
PIVOT_SPEED  = 0.18
PIVOT_DUR_S  = 1.8


# ══════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def wrap(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def astar(grid, start, goal):
    rows, cols = grid.shape
    def h(r, c): return math.hypot(r - goal[0], c - goal[1])
    heap = [(h(*start), 0.0, start)]
    came = {}
    g    = {start: 0.0}
    seen = set()
    dirs  = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    costs = [1.0,   1.0,  1.0,   1.0,  1.414,   1.414, 1.414, 1.414]
    limit = GRID_CELLS * GRID_CELLS // 2
    n = 0
    while heap and n < limit:
        n += 1
        _, gc, cur = heappop(heap)
        if cur in seen: continue
        seen.add(cur)
        if cur == goal:
            path = []
            while cur in came: path.append(cur); cur = came[cur]
            return list(reversed(path))
        for (dr, dc), cost in zip(dirs, costs):
            nb = (cur[0]+dr, cur[1]+dc)
            if not (0 <= nb[0] < rows and 0 <= nb[1] < cols): continue
            if grid[nb[0], nb[1]] >= 50: continue
            ng = gc + cost
            if nb not in g or ng < g[nb]:
                g[nb] = ng; came[nb] = cur
                heappush(heap, (ng + h(*nb), ng, nb))
    return None


class Odometer:
    def __init__(self):
        self._lock   = threading.Lock()
        self._driven = 0.0
        self._active = False
        self._last_t = time.monotonic()

    def reset(self):
        with self._lock:
            self._driven = 0.0
            self._last_t = time.monotonic()
            self._active = True

    def pause(self):
        with self._lock: self._active = False

    def update(self, speed_ms: float):
        now = time.monotonic()
        with self._lock:
            dt = now - self._last_t
            self._last_t = now
            if self._active and 0 < dt < 0.5:
                self._driven += max(0.0, speed_ms) * dt   # only count forward motion

    @property
    def driven_m(self):
        with self._lock: return self._driven


# ══════════════════════════════════════════════════════════════════════════════
#  NODE
# ══════════════════════════════════════════════════════════════════════════════

class NavDepthProcessor(Node):

    def __init__(self):
        super().__init__('nav_depth_processor')

        self.declare_parameter('linear_speed',  MAX_LINEAR)
        self.declare_parameter('angular_speed', MAX_ANGULAR)
        self.declare_parameter('safety_radius', SAFETY_RADIUS_M)
        self.declare_parameter('lookahead',     LOOKAHEAD_M)
        self.declare_parameter('right_flip',    RIGHT_FLIP_COMPENSATION)
        self.declare_parameter('reverse_linear', REVERSE_LINEAR)

        self._lin_spd      = self.get_parameter('linear_speed').value
        self._ang_spd      = self.get_parameter('angular_speed').value
        self._safety_r     = self.get_parameter('safety_radius').value
        self._saf_cells    = int(self._safety_r / GRID_RES_M)
        self._lookahead    = self.get_parameter('lookahead').value
        self._right_flip   = self.get_parameter('right_flip').value
        self._rev_lin      = self.get_parameter('reverse_linear').value

        # state
        self._lock       = threading.Lock()
        self._bridge     = CvBridge()
        self._depth_lock = threading.Lock()
        self._depth_img  = None

        # pose
        self._x = self._y = self._yaw = 0.0
        self._odom_ok   = False
        self._imu_yawrt = 0.0
        self._imu_ok    = False
        self._last_dr_t = self.get_clock().now()

        # camera
        self._fx = self._fy = self._cx = self._cy = None
        self._img_w = 424; self._img_h = 240

        # goal
        self._goal_xy    = None
        self._goal_depth = None
        self._odo        = Odometer()

        # path
        self._path     = []
        self._path_idx = 0

        # steering state (for PD controller)
        self._prev_yaw_err = 0.0
        self._prev_ctrl_t  = time.monotonic()

        # misc
        self._manual_obs = []
        self._state      = 'IDLE'
        self._active     = False
        self._pivot_t0   = 0.0
        self._pivot_dir  = 1.0
        self._stuck_t    = time.monotonic()
        self._stuck_pos  = (0.0, 0.0)
        self._last_lin   = 0.0
        self._last_ang   = 0.0
        self._ctrl_n     = 0

        # QoS
        be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # subscribers
        self.create_subscription(Image,
            '/camera/camera/aligned_depth_to_color/image_raw', self._cb_depth, be)
        self.create_subscription(CameraInfo,
            '/camera/camera/color/camera_info', self._cb_caminfo, rel)
        self.create_subscription(Odometry,
            '/nav/depth_odom', self._cb_odom, rel)
        self.create_subscription(Float32MultiArray,
            '/nav/goal_camera_frame', self._cb_goal, rel)
        self.create_subscription(Bool,
            '/nav/cancel', self._cb_cancel, rel)
        self.create_subscription(Bool,
            '/nav/joystick_active', self._cb_joy, rel)
        self.create_subscription(Float32MultiArray,
            '/nav/manual_obstacles', self._cb_manual_obs, rel)
        self.create_subscription(Float32MultiArray,
            '/imu/gyro_deg_s', self._cb_imu, rel)

        # publishers
        self._pub_cmd      = self.create_publisher(Twist,       '/nav/cmd_vel',          rel)
        self._pub_path_msg = self.create_publisher(Path,        '/nav/planned_path',     rel)
        self._pub_grid     = self.create_publisher(MarkerArray, '/nav/obstacle_markers', rel)
        self._pub_stat     = self.create_publisher(String,      '/nav/status',           rel)
        self._pub_prog     = self.create_publisher(String,      '/nav/path_progress',    rel)
        self._pub_viz      = self.create_publisher(MarkerArray, '/nav/viz_markers',      rel)

        self._tf_br = TransformBroadcaster(self) if _HAS_TF2 else None

        # timers — replan on its OWN timer, never blocks control loop
        self.create_timer(1.0 / REPLAN_HZ,  self._timer_replan)
        self.create_timer(1.0 / CONTROL_HZ, self._timer_control)
        self.create_timer(0.05,             self._timer_dr)
        self.create_timer(0.20,             self._timer_progress)
        self.create_timer(0.05,             self._timer_viz)

        self.get_logger().info('nav_depth_processor v4  ready')
        self.get_logger().info(
            f'  right_flip={self._right_flip}  reverse_linear={self._rev_lin}  '
            f'lookahead={self._lookahead}m')
        self.get_logger().info(
            f'  deadband={HEADING_DEADBAND_DEG}°  centre_zone=±{CENTRE_ZONE_DEG:.1f}°  '
            f'K_P={K_P}  K_D={K_D}')
        self.get_logger().info(
            f'  odo_margin={GOAL_ODOMETER_MARGIN}m  '
            f'cam_emerg={GOAL_CAM_EMERGENCY_M}m  dr_tol={GOAL_DR_TOL_M}m')

    # ── sensor callbacks ──────────────────────────────────────────────────────

    def _cb_caminfo(self, msg):
        if self._fx is not None: return
        self._fx = msg.k[0]; self._fy = msg.k[4]
        self._cx = msg.k[2]; self._cy = msg.k[5]
        self._img_w = msg.width; self._img_h = msg.height

    def _cb_depth(self, msg):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
            with self._depth_lock:
                self._depth_img = img.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)

    def _cb_imu(self, msg):
        if len(msg.data) >= 3:
            self._imu_yawrt = math.radians(float(msg.data[2]))
            self._imu_ok    = True

    def _cb_odom(self, msg):
        with self._lock:
            self._x   = msg.pose.pose.position.x
            self._y   = msg.pose.pose.position.y
            qz        = msg.pose.pose.orientation.z
            qw        = msg.pose.pose.orientation.w
            self._yaw = 2.0 * math.atan2(qz, qw)
            self._odom_ok = True

    def _cb_joy(self, msg):
        if msg.data:
            self._last_dr_t = self.get_clock().now()
            self._odo.pause()

    def _cb_manual_obs(self, msg):
        d = list(msg.data)
        with self._lock:
            self._manual_obs = [(float(d[i]), float(d[i+1]), float(d[i+2]))
                                for i in range(0, len(d)-2, 3)]

    # ── goal / cancel ─────────────────────────────────────────────────────────

    def _cb_goal(self, msg):
        if len(msg.data) < 3: return
        cx, cy, cz = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
        print(f'[GOAL] cam=({cx:.3f},{cy:.3f},{cz:.3f})', flush=True)

        tilt   = CAM_TILT_RAD
        body_x =  cz * math.cos(-tilt) - cy * math.sin(-tilt) + 0.20
        body_y = -cx

        with self._lock:
            yaw = self._yaw; rx = self._x; ry = self._y

        gx = rx + body_x * math.cos(yaw) - body_y * math.sin(yaw)
        gy = ry + body_x * math.sin(yaw) + body_y * math.cos(yaw)

        stop_dist = max(0.0, cz - GOAL_ODOMETER_MARGIN)
        print(f'[GOAL] world=({gx:.2f},{gy:.2f})  '
              f'depth={cz:.2f}m  stop_after={stop_dist:.2f}m', flush=True)

        with self._lock:
            self._goal_xy    = (gx, gy)
            self._goal_depth = cz
            self._path       = []
            self._path_idx   = 0
            self._state      = 'NAVIGATING'
            self._active     = True
            self._stuck_t    = time.monotonic()
            self._stuck_pos  = (rx, ry)

        self._prev_yaw_err = 0.0
        self._odo.reset()
        self._pub_status('NAVIGATING')

    def _cb_cancel(self, msg):
        if msg.data:
            with self._lock:
                self._active = False; self._state = 'IDLE'; self._path = []
            self._odo.pause()
            self._stop()
            self._pub_status('IDLE')

    # ── dead reckoning ────────────────────────────────────────────────────────

    def _timer_dr(self):
        if self._odom_ok: return
        now = self.get_clock().now()
        dt  = (now - self._last_dr_t).nanoseconds / 1e9
        self._last_dr_t = now
        if not (0 < dt < 0.5): return
        with self._lock:
            self._yaw += (self._imu_yawrt if self._imu_ok else self._last_ang) * dt
            self._yaw  = wrap(self._yaw)
            self._x   += self._last_lin * math.cos(self._yaw) * dt
            self._y   += self._last_lin * math.sin(self._yaw) * dt

    # ── occupancy grid ────────────────────────────────────────────────────────

    def _build_grid(self):
        with self._depth_lock:
            if self._depth_img is None: return None
            depth = self._depth_img.copy()

        fx = self._fx or 213.0; fy = self._fy or 213.0
        cx = self._cx or 212.0; cy = self._cy or 120.0
        h, w   = depth.shape
        grid   = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.int8)
        origin = GRID_CELLS // 2

        ri, ci = np.mgrid[0:h, 0:w]
        z = depth[ri, ci]
        ok = (z > BLIND_ZONE_M) & (z < GRID_RANGE_M * 1.2) & (z > 0)
        zv = z[ok]; rv = ri[ok]; cv_ = ci[ok]

        cam_x = (cv_ - cx) / fx * zv
        cam_y = (rv  - cy) / fy * zv

        tilt = CAM_TILT_RAD
        wh   = zv  * math.sin(tilt) + cam_y * math.cos(tilt) + CAM_HEIGHT_M
        wf   = zv  * math.cos(tilt) - cam_y * math.sin(tilt)
        wl   = -cam_x

        rock = (wh > ROCK_HEIGHT_MIN_M) & (wh < ROCK_HEIGHT_MAX_M)
        gr   = origin - (wf[rock] / GRID_RES_M).astype(int)
        gc   = origin + (wl[rock] / GRID_RES_M).astype(int)
        m    = (gr >= 0) & (gr < GRID_CELLS) & (gc >= 0) & (gc < GRID_CELLS)
        grid[gr[m], gc[m]] = 100

        blind_cells = int(BLIND_ZONE_M / GRID_RES_M)
        br0 = max(0, origin - blind_cells)
        grid[br0:origin+1, :] = 0

        if self._saf_cells > 0:
            k  = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self._saf_cells*2+1, self._saf_cells*2+1))
            gi = cv2.dilate((grid > 50).astype(np.uint8), k)
            grid[gi > 0] = 100
            grid[br0:origin+1, :] = 0

        with self._lock: manual = list(self._manual_obs)
        for (mx, my, mz) in manual:
            bx =  mz * math.cos(-tilt) - my * math.sin(-tilt)
            by = -mx
            r_ = origin - int(bx / GRID_RES_M)
            c_ = origin + int(by / GRID_RES_M)
            if 0 <= r_ < GRID_CELLS and 0 <= c_ < GRID_CELLS:
                grid[max(0,r_-3):min(GRID_CELLS,r_+4),
                     max(0,c_-3):min(GRID_CELLS,c_+4)] = 100
        return grid

    # ── replan (own timer, never blocks control) ──────────────────────────────

    def _timer_replan(self):
        with self._lock:
            active       = self._active; state = self._state
            gxy          = self._goal_xy
            rx, ry, ryaw = self._x, self._y, self._yaw

        if not active or state not in ('NAVIGATING', 'STUCK'): return
        if gxy is None: return

        gx, gy    = gxy
        dist_goal = math.hypot(gx-rx, gy-ry)
        if dist_goal < GOAL_DR_TOL_M: return

        grid = self._build_grid()
        if grid is None: return

        self._pub_grid_markers(grid, rx, ry, ryaw)

        origin = GRID_CELLS // 2
        dx_r   =  (gx-rx)*math.cos(-ryaw) - (gy-ry)*math.sin(-ryaw)
        dy_r   =  (gx-rx)*math.sin(-ryaw) + (gy-ry)*math.cos(-ryaw)

        goal_r = int(np.clip(origin - int(dx_r/GRID_RES_M), 1, GRID_CELLS-2))
        goal_c = int(np.clip(origin + int(dy_r/GRID_RES_M), 1, GRID_CELLS-2))

        if grid[goal_r, goal_c] >= 50:
            free = self._nearest_free(grid, goal_r, goal_c)
            if free is None:
                print('[PLAN] goal blocked', flush=True); return
            goal_r, goal_c = free

        path_rc = astar(grid, (origin, origin), (goal_r, goal_c))
        if path_rc is None:
            print('[PLAN] no path', flush=True)
            with self._lock: self._state = 'STUCK'
            return

        path_w = []
        for pr, pc in [(origin, origin)] + path_rc:
            lx =  (origin-pr) * GRID_RES_M
            ly =  (pc-origin) * GRID_RES_M
            wx = rx + lx*math.cos(ryaw) - ly*math.sin(ryaw)
            wy = ry + lx*math.sin(ryaw) + ly*math.cos(ryaw)
            path_w.append((wx, wy))

        # Preserve progress: resume from closest point on new path
        best_i = min(
            range(len(path_w)),
            key=lambda i: math.hypot(path_w[i][0]-rx, path_w[i][1]-ry))

        with self._lock:
            self._path     = path_w
            self._path_idx = best_i
            if self._state == 'STUCK': self._state = 'NAVIGATING'

        print(f'[PLAN] {len(path_w)} wpts  dist={dist_goal:.2f}m  '
              f'resume={best_i}  odo={self._odo.driven_m:.2f}m', flush=True)
        self._pub_path(path_w)

    # ── control loop (10 Hz — only reads path, zero blocking) ────────────────

    def _timer_control(self):
        with self._lock:
            active       = self._active; state = self._state
            path         = list(self._path); pidx = self._path_idx
            rx, ry, ryaw = self._x, self._y, self._yaw
            gxy          = self._goal_xy; gdepth = self._goal_depth

        if not active: return

        self._odo.update(self._last_lin)
        driven = self._odo.driven_m
        gx, gy = gxy if gxy else (0.0, 0.0)

        # ── Goal stop ─────────────────────────────────────────────────────
        # Primary: odometer
        odo_stop = gdepth is not None and driven >= (gdepth - GOAL_ODOMETER_MARGIN)

        # Emergency: camera sees something very close ahead
        cam_fwd  = self._sample_fwd_depth()
        cam_stop = cam_fwd < GOAL_CAM_EMERGENCY_M

        # Fallback: DR world distance
        dist_dr  = math.hypot(gx-rx, gy-ry)
        dr_stop  = dist_dr < GOAL_DR_TOL_M

        if odo_stop or cam_stop or dr_stop:
            why = ('odometer' if odo_stop else
                   'camera_emergency' if cam_stop else 'dead-reckoning')
            print(f'[GOAL] REACHED ({why})  '
                  f'driven={driven:.2f}m  cam={cam_fwd:.2f}m  dr={dist_dr:.2f}m',
                  flush=True)
            self._arrive()
            return

        # ── Stuck escape ──────────────────────────────────────────────────
        if state == 'STUCK':
            now = time.monotonic()
            if self._pivot_t0 == 0: self._pivot_t0 = now
            if now - self._pivot_t0 < PIVOT_DUR_S:
                self._send(0.0, PIVOT_SPEED * self._pivot_dir)
            else:
                self._pivot_t0 = 0
                with self._lock: self._state = 'NAVIGATING'
            return

        if not path: return

        # ── Advance waypoint index ────────────────────────────────────────
        while pidx < len(path) - 1:
            if math.hypot(path[pidx][0]-rx, path[pidx][1]-ry) < WAYPOINT_TOL_M:
                pidx += 1
            else: break
        with self._lock: self._path_idx = pidx

        # ── Stuck detection ───────────────────────────────────────────────
        moved = math.hypot(rx-self._stuck_pos[0], ry-self._stuck_pos[1])
        if moved > STUCK_DIST_M:
            self._stuck_t = time.monotonic(); self._stuck_pos = (rx, ry)
        elif time.monotonic() - self._stuck_t > STUCK_TIME_S:
            with self._lock: self._state = 'STUCK'
            self.get_logger().warn('Stuck — escape pivot')
            return

        # ── Pure pursuit target ───────────────────────────────────────────
        target = None
        for i in range(pidx, len(path)):
            if math.hypot(path[i][0]-rx, path[i][1]-ry) >= self._lookahead:
                target = path[i]; break
        if target is None: target = path[-1]

        tx, ty  = target
        desired = math.atan2(ty-ry, tx-rx)
        yaw_err = wrap(desired - ryaw)
        yaw_abs = abs(yaw_err)
        yaw_deg = math.degrees(yaw_abs)

        d_target = math.hypot(tx-rx, ty-ry)

        # ── Angular output with deadband + PD + centre-zone suppression ───

        now_t = time.monotonic()
        dt_ctrl = max(now_t - self._prev_ctrl_t, 0.001)
        self._prev_ctrl_t = now_t

        # Deadband: below threshold, no turn needed
        in_deadband = yaw_deg < HEADING_DEADBAND_DEG

        # Centre-zone: target is close AND within ±CENTRE_ZONE_DEG of straight ahead
        in_centre_zone = (yaw_deg < CENTRE_ZONE_DEG) and (d_target < CENTRE_ZONE_M)

        if in_deadband or in_centre_zone:
            ang = 0.0
            self._prev_yaw_err = yaw_err   # don't accumulate D term during suppression
        else:
            # P term
            p = K_P * yaw_err
            # D term (rate of change of error)
            d_err = (yaw_err - self._prev_yaw_err) / dt_ctrl
            d     = K_D * d_err
            ang   = float(np.clip(p - d, -self._ang_spd, self._ang_spd))
            self._prev_yaw_err = yaw_err

        # ── Linear speed (reduce during large heading errors) ─────────────
        if yaw_deg >= HEADING_STOP_DEG:
            lin_frac = TURN_FWD_FLOOR
        elif yaw_deg >= HEADING_SLOW_DEG:
            t_       = (yaw_deg - HEADING_SLOW_DEG) / (HEADING_STOP_DEG - HEADING_SLOW_DEG)
            lin_frac = 1.0 - t_ * (1.0 - TURN_FWD_FLOOR)
        else:
            lin_frac = 1.0

        lin = self._lin_spd * lin_frac
        if d_target < 1.0:
            lin = max(lin * (d_target / 1.0), MIN_CREEP)

        self._ctrl_n += 1
        if self._ctrl_n % 10 == 1:
            zone = 'DEADBAND' if in_deadband else 'CENTRE' if in_centre_zone else 'STEER'
            print(f'[CTRL] {zone}  err={yaw_deg:+.1f}°  '
                  f'lin={lin:.3f}  ang={ang:+.3f}  '
                  f'odo={driven:.2f}m  cam={cam_fwd:.2f}m  wp={pidx}/{len(path)}',
                  flush=True)

        self._send(lin, ang)

    # ── camera forward depth ──────────────────────────────────────────────────

    def _sample_fwd_depth(self) -> float:
        """
        Median depth of a small centre-column patch across the full image height.
        With 25° downward tilt this sees ground directly ahead at various distances.
        Returns 99.0 if no valid reading.
        """
        with self._depth_lock:
            if self._depth_img is None: return 99.0
            d = self._depth_img.copy()
        h, w = d.shape
        # Narrow vertical strip in the centre third horizontally
        patch = d[:, int(w*0.35):int(w*0.65)]
        valid = patch[(patch > 0.15) & (patch < 5.0)]
        return float(np.median(valid)) if valid.size >= 10 else 99.0

    # ── motor output ──────────────────────────────────────────────────────────

    def _send(self, lin: float, ang: float):
        """
        Applies hardware compensation flags before publishing.
        REVERSE_LINEAR negates linear.x  (fixes driving-backwards bug).
        RIGHT_FLIP_COMPENSATION negates angular.z (fixes turn direction).
        """
        t = Twist()
        lin_out = -lin if self._rev_lin   else lin
        ang_out = -ang if self._right_flip else ang
        t.linear.x  = float(np.clip(lin_out, -self._lin_spd, self._lin_spd))
        t.angular.z = float(np.clip(ang_out, -self._ang_spd, self._ang_spd))
        self._last_lin = lin   # store pre-negation value for odometer + DR
        self._last_ang = ang
        self._pub_cmd.publish(t)

    def _stop(self):
        self._pub_cmd.publish(Twist())
        self._pub_cmd.publish(Twist())
        self._last_lin = 0.0; self._last_ang = 0.0

    def _arrive(self):
        with self._lock: self._active = False; self._state = 'IDLE'
        self._odo.pause()
        self._stop()
        self.get_logger().info('Goal reached')
        self._pub_status('GOAL_REACHED')

    # ── TF + viz ──────────────────────────────────────────────────────────────

    def _timer_viz(self):
        with self._lock:
            rx,ry,ryaw = self._x, self._y, self._yaw
            path = list(self._path); pidx = self._path_idx
            gxy  = self._goal_xy; active = self._active
        stamp = self.get_clock().now().to_msg()

        if self._tf_br:
            tf = TransformStamped()
            tf.header.stamp = stamp; tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = rx; tf.transform.translation.y = ry
            tf.transform.rotation.z = math.sin(ryaw/2)
            tf.transform.rotation.w = math.cos(ryaw/2)
            self._tf_br.sendTransform(tf)

        ma  = MarkerArray(); mid = 0
        def mk(ns, typ):
            nonlocal mid
            m = Marker(); m.header.stamp = stamp; m.header.frame_id = 'odom'
            m.ns = ns; m.id = mid; m.type = typ; m.action = Marker.ADD
            m.pose.orientation.w = 1.0; m.lifetime.sec = 1; mid += 1
            return m

        rp = mk('rover', Marker.CYLINDER)
        rp.pose.position.x = rx; rp.pose.position.y = ry; rp.pose.position.z = 0.05
        rp.pose.orientation.z = math.sin(ryaw/2); rp.pose.orientation.w = math.cos(ryaw/2)
        rp.scale.x = rp.scale.y = 0.40; rp.scale.z = 0.10
        rp.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.5); ma.markers.append(rp)

        if active and path and pidx < len(path):
            ln = mk('path', Marker.LINE_STRIP)
            ln.scale.x = 0.04; ln.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)
            ln.points.append(Point(x=rx, y=ry, z=0.03))
            for wx,wy in path[pidx:]:
                ln.points.append(Point(x=float(wx), y=float(wy), z=0.03))
            if len(ln.points) >= 2: ma.markers.append(ln)

        if gxy and active:
            gs = mk('goal', Marker.SPHERE)
            gs.pose.position.x=gxy[0]; gs.pose.position.y=gxy[1]; gs.pose.position.z=0.15
            gs.scale.x=gs.scale.y=gs.scale.z=0.30
            gs.color=ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.8); ma.markers.append(gs)

        self._pub_viz.publish(ma)

    # ── progress publisher ────────────────────────────────────────────────────

    def _timer_progress(self):
        import json as _j
        with self._lock:
            path  = list(self._path); pidx = self._path_idx
            rx,ry,ryaw = self._x, self._y, self._yaw
            state = self._state; active = self._active
            gx,gy = self._goal_xy if self._goal_xy else (None, None)
        if not path or gx is None:
            m = String(); m.data = _j.dumps({'state':state,'active':active})
            self._pub_prog.publish(m); return
        dr  = sum(math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])
                  for i in range(max(1,pidx), len(path))) if len(path)>1 else 0.0
        dg  = math.hypot(gx-rx, gy-ry)
        cte = 0.0
        if 0 < pidx < len(path):
            ax,ay = path[pidx-1]; bx,by = path[pidx]
            abx = bx-ax; aby = by-ay; ab2 = max(abx**2+aby**2, 1e-9)
            t   = max(0.0, min(1.0, ((rx-ax)*abx+(ry-ay)*aby)/ab2))
            cte = math.hypot(rx-ax-t*abx, ry-ay-t*aby)
        data = {'state':state, 'active':active,
                'driven_m':round(self._odo.driven_m,2),
                'dist_remaining_m':round(dr,2), 'dist_to_goal_m':round(dg,2),
                'waypoint_idx':pidx, 'total_waypoints':len(path),
                'cte_m':round(cte,3)}
        m = String(); m.data = _j.dumps(data)
        self._pub_prog.publish(m)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _pub_path(self, path_w):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for wx, wy in path_w:
            ps = PoseStamped(); ps.header = msg.header
            ps.pose.position.x = float(wx); ps.pose.position.y = float(wy)
            ps.pose.position.z = 0.03; ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._pub_path_msg.publish(msg)

    def _pub_grid_markers(self, grid, rx, ry, ryaw):
        ma = MarkerArray(); stamp = self.get_clock().now().to_msg()
        origin = GRID_CELLS // 2
        clr = Marker(); clr.header.stamp = stamp; clr.header.frame_id = 'odom'
        clr.ns = 'obs'; clr.id = 0; clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        mid = 1
        for pr, pc in zip(*np.where(grid >= 50)):
            lx = (origin-pr)*GRID_RES_M; ly = (pc-origin)*GRID_RES_M
            wx = rx + lx*math.cos(ryaw) - ly*math.sin(ryaw)
            wy = ry + lx*math.sin(ryaw) + ly*math.cos(ryaw)
            m = Marker(); m.header.stamp = stamp; m.header.frame_id = 'odom'
            m.ns = 'obs'; m.id = mid; m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = float(wx); m.pose.position.y = float(wy)
            m.pose.position.z = 0.15; m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = float(GRID_RES_M); m.scale.z = 0.30
            m.color.r = 1.0; m.color.g = 0.2; m.color.a = 0.6; m.lifetime.sec = 1
            ma.markers.append(m); mid += 1
        self._pub_grid.publish(ma)

    def _pub_status(self, s):
        m = String(); m.data = s; self._pub_stat.publish(m)

    def _nearest_free(self, grid, r, c, limit=25):
        from collections import deque
        q = deque([(r,c)]); seen = {(r,c)}; R,C = grid.shape
        while q:
            cr,cc = q.popleft()
            if grid[cr,cc] < 50: return (cr,cc)
            for dr,dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr,nc = cr+dr, cc+dc
                if ((nr,nc) not in seen and 0<=nr<R and 0<=nc<C
                        and abs(nr-r)+abs(nc-c) < limit):
                    seen.add((nr,nc)); q.append((nr,nc))
        return None


# ══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = NavDepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == '__main__':
    main()

