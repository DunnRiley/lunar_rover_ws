#!/usr/bin/env python3
"""
nav_depth_processor.py  —  runs on MINI PC

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  OVERVIEW
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  1. Subscribes to the RAW depth image from D435
     (/camera/camera/aligned_depth_to_color/image_raw)

  2. Receives a NavGoal message from the laptop
     (goal is a 3D point in camera frame, computed by nav_goal_relay.py)

  3. Slices the depth image into a horizontal band that corresponds
     to rock-height obstacles given the camera geometry:
       - Camera height:  0.70 m
       - Camera tilt:   -25 deg (looking down)
       - Rock height:    0.30-0.40 m
       - Blind zone:     ~0.70 m in front of rover (not seen by camera)

  4. Projects that slice into a 2D top-down occupancy grid
     (bird's-eye view, robot at origin)

  5. Inflates obstacles by SAFETY_RADIUS

  6. Runs A* from robot pose to goal

  7. Publishes:
       /nav/cmd_vel        geometry_msgs/Twist   (motor commands)
       /nav/planned_path   nav_msgs/Path         (for RViz on laptop)
       /nav/occupancy_grid nav_msgs/OccupancyGrid (for RViz debug)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  DEAD RECKONING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Without encoders, pose is integrated from cmd_vel.
  This drifts over time but is good enough for short runs.
  When D435 visual odometry is added later, swap in the odom
  topic by setting use_visual_odom:=true.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  CAMERA GEOMETRY (D435, 424x240, ~69 deg HFOV, ~42 deg VFOV)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  The D435 at 25 deg downward tilt, 70 cm high:
    Ground directly below camera: 0 m forward
    Horizon (flat ground):        ~1.5 m forward at bottom of image
    Top of 35 cm rock at 1.5 m:  appears ~60% down the image

  We scan rows that correspond to 0.15-0.50 m height above ground
  at the range seen in each depth pixel, which catches rocks without
  triggering on the ground plane itself.
"""

import math
import time
import threading
from collections import deque
from heapq import heappush, heappop

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point, Vector3, Quaternion
from std_msgs.msg import Bool, String, Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    _HAS_TF2 = True
except ImportError:
    _HAS_TF2 = False
    print('[WARN] tf2_ros not available — TF will not be broadcast', flush=True)



# ══════════════════════════════════════════════════════════════════════════════
#  TUNABLE CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════

# Camera physical setup
CAM_HEIGHT_M      = 0.70    # metres above ground
CAM_TILT_DEG      = -25.0   # negative = looking down
CAM_TILT_RAD      = math.radians(CAM_TILT_DEG)

# Obstacle detection band (in world height, metres above ground)
ROCK_HEIGHT_MIN_M = 0.10    # ignore anything below this (ground noise)
ROCK_HEIGHT_MAX_M = 0.55    # ignore anything above this (too tall = not a rock)

# Blind zone: depth pixels closer than this are treated as obstacle-free
# because the rover body blocks the view and we've been told the first
# 0.7 m is clear at start.
BLIND_ZONE_M      = 0.70

# Grid parameters
GRID_RES_M        = 0.05    # metres per grid cell  (5 cm)
GRID_RANGE_M      = 4.0     # half-width of square grid around robot
GRID_CELLS        = int(2 * GRID_RANGE_M / GRID_RES_M)   # 160 cells

# Safety inflation around obstacles
SAFETY_RADIUS_M   = 0.80    # metres
SAFETY_CELLS      = int(SAFETY_RADIUS_M / GRID_RES_M)

# Navigation control
LOOKAHEAD_M       = 0.60    # pure-pursuit lookahead distance
GOAL_TOL_M        = 0.30    # "close enough" to goal
WAYPOINT_TOL_M    = 0.25    # advance to next waypoint when within this

# Speed limits
MAX_LINEAR        = 0.35    # m/s  (~half of rover's max)
MAX_ANGULAR       = 0.60    # rad/s
ARC_BLEND         = 0.55    # how much angular correction to blend in

# Replan rate
REPLAN_HZ         = 2.0     # how often to re-run A* (Hz)
CONTROL_HZ        = 10.0    # cmd_vel publish rate (Hz)

# Stuck detection
STUCK_TIME_S      = 4.0     # if no progress for this long → try pivot
STUCK_DIST_M      = 0.10    # "no progress" threshold

# Pivot escape
PIVOT_SPEED       = 0.25    # angular speed for escape pivot
PIVOT_DURATION_S  = 1.2     # how long to pivot before replanning


# ══════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def astar(grid: np.ndarray, start: tuple, goal: tuple):
    """
    A* on a 2-D numpy occupancy grid (0=free, 100=occupied).
    start / goal are (row, col) integer tuples.
    Returns list of (row, col) or None if no path found.
    """
    rows, cols = grid.shape

    def h(r, c):
        return math.sqrt((r - goal[0])**2 + (c - goal[1])**2)

    open_set = []
    heappush(open_set, (h(*start), 0.0, start))
    came_from = {}
    g = {start: 0.0}
    visited = set()

    dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    costs = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]

    iters = 0
    max_iters = GRID_CELLS * GRID_CELLS // 2

    while open_set and iters < max_iters:
        iters += 1
        _, gc, cur = heappop(open_set)
        if cur in visited:
            continue
        visited.add(cur)

        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return list(reversed(path))

        for (dr, dc), cost in zip(dirs, costs):
            nr, nc = cur[0] + dr, cur[1] + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if grid[nr, nc] >= 50:
                continue
            ng = gc + cost
            nb = (nr, nc)
            if nb not in g or ng < g[nb]:
                g[nb] = ng
                came_from[nb] = cur
                heappush(open_set, (ng + h(nr, nc), ng, nb))

    return None


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN NODE
# ══════════════════════════════════════════════════════════════════════════════

class NavDepthProcessor(Node):

    def __init__(self):
        super().__init__('nav_depth_processor')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('use_visual_odom', False)
        self.declare_parameter('linear_speed',    MAX_LINEAR)
        self.declare_parameter('angular_speed',   MAX_ANGULAR)
        self.declare_parameter('goal_tolerance',  GOAL_TOL_M)
        self.declare_parameter('safety_radius',   SAFETY_RADIUS_M)

        self._lin_speed    = self.get_parameter('linear_speed').value
        self._ang_speed    = self.get_parameter('angular_speed').value
        self._goal_tol     = self.get_parameter('goal_tolerance').value
        self._safety_r     = self.get_parameter('safety_radius').value
        self._safety_cells = int(self._safety_r / GRID_RES_M)

        # ── State ────────────────────────────────────────────────────────
        self._lock          = threading.Lock()
        self._bridge        = CvBridge()

        # Pose — updated by depth odom node if available, else cmd_vel DR
        self._x     = 0.0
        self._y     = 0.0
        self._yaw   = 0.0
        self._last_dr_time   = self.get_clock().now()
        self._odom_available = False   # set True when /nav/depth_odom arrives

        # Camera intrinsics (filled by camera_info callback)
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None
        self._img_w = 424
        self._img_h = 240

        # Latest depth image
        self._depth_img: np.ndarray | None = None
        self._depth_lock = threading.Lock()

        # Goal in world frame (x forward, y left)
        self._goal_world: tuple | None = None   # (gx, gy)

        # Manual obstacle hints in CAMERA FRAME from GUI (no world-frame drift)
        self._manual_obstacle_cam: list = []    # list of (cam_x, cam_y, cam_z)

        # Current A* path (list of (world_x, world_y))
        self._path: list = []
        self._path_idx  = 0

        # Stuck detection
        self._last_progress_time = time.monotonic()
        self._last_progress_pos  = (0.0, 0.0)

        # State machine
        self._state = 'IDLE'   # IDLE | NAVIGATING | PIVOTING | STUCK

        self._pivot_start  = 0.0
        self._pivot_dir    = 1.0

        # Active navigation flag
        self._active = False
        self._ctrl_print_n = 0   # throttle control-loop terminal output

        # ── QoS ──────────────────────────────────────────────────────────
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        rel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ── Subscribers ──────────────────────────────────────────────────
        # Raw depth from D435 (on miniPC, always available)
        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self._depth_cb, be_qos)

        # Camera intrinsics
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._caminfo_cb, rel_qos)

        # Pose from depth-flow odometry node (preferred over cmd_vel DR)
        from nav_msgs.msg import Odometry as _Odom
        self.create_subscription(
            _Odom,
            '/nav/depth_odom',
            self._odom_cb, rel_qos)

        # Goal from laptop relay (Float32MultiArray: [cam_x, cam_y, cam_z])
        self.create_subscription(
            Float32MultiArray,
            '/nav/goal_camera_frame',
            self._goal_cb, rel_qos)

        # Cancel navigation
        self.create_subscription(
            Bool,
            '/nav/cancel',
            self._cancel_cb, rel_qos)

        # Joystick activity signal from mux (lets us pause dead reckoning)
        self.create_subscription(
            Bool,
            '/nav/joystick_active',
            self._joy_active_cb, rel_qos)

        # Manual obstacle hints from nav_control_panel GUI
        # Float32MultiArray: [cam_x, cam_y, cam_z,  cam_x, cam_y, cam_z, ...]
        self.create_subscription(
            Float32MultiArray,
            '/nav/manual_obstacles',
            self._manual_obstacles_cb, rel_qos)

        # ── Publishers ───────────────────────────────────────────────────
        self._cmd_pub      = self.create_publisher(Twist,       '/nav/cmd_vel',          rel_qos)
        self._path_pub     = self.create_publisher(Path,        '/nav/planned_path',     rel_qos)
        self._grid_pub     = self.create_publisher(MarkerArray, '/nav/obstacle_markers', rel_qos)
        self._stat_pub     = self.create_publisher(String,      '/nav/status',           rel_qos)
        self._progress_pub = self.create_publisher(String,      '/nav/path_progress',    rel_qos)
        # Rich RViz visualisation markers
        self._viz_pub      = self.create_publisher(MarkerArray, '/nav/viz_markers',      rel_qos)

        # TF broadcaster — publishes odom→base_link so RViz fixed frame works
        if _HAS_TF2:
            self._tf_br = TransformBroadcaster(self)
        else:
            self._tf_br = None

        # ── Timers ───────────────────────────────────────────────────────
        self._replan_timer   = self.create_timer(1.0 / REPLAN_HZ,  self._replan_cb)
        self._control_timer  = self.create_timer(1.0 / CONTROL_HZ, self._control_cb)
        self._dr_timer       = self.create_timer(0.05,              self._dr_update)
        self._progress_timer = self.create_timer(0.20,              self._progress_cb)   # 5 Hz
        self._tf_viz_timer   = self.create_timer(0.05,              self._tf_viz_cb)     # 20 Hz

        self.get_logger().info('='*60)
        self.get_logger().info('  nav_depth_processor  READY')
        self.get_logger().info('  Waiting for goal on /nav/goal_camera_frame')
        self.get_logger().info('='*60)

    # ── Camera info ───────────────────────────────────────────────────────

    def _caminfo_cb(self, msg: CameraInfo):
        if self._fx is not None:
            return  # already have it
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self._img_w = msg.width
        self._img_h = msg.height
        self.get_logger().info(
            f'Camera intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}  {self._img_w}x{self._img_h}')

    # ── Depth image ───────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # D435 depth is 16UC1 in millimetres
            with self._depth_lock:
                self._depth_img = img.astype(np.float32) / 1000.0  # → metres
        except Exception as e:
            self.get_logger().error(f'Depth decode error: {e}',
                                    throttle_duration_sec=5.0)

    # ── Goal callback ─────────────────────────────────────────────────────

    def _goal_cb(self, msg: Float32MultiArray):
        """
        Goal arrives as [cam_x, cam_y, cam_z] in D435 camera frame.
        Camera frame: x=right, y=down, z=forward.
        We convert to world frame relative to current robot pose.
        """
        if len(msg.data) < 3:
            print('[GOAL] ✗  message too short:', list(msg.data), flush=True)
            return

        cx, cy, cz = msg.data[0], msg.data[1], msg.data[2]
        print(f'[GOAL] received  cam=({cx:.3f}, {cy:.3f}, {cz:.3f})', flush=True)

        tilt = CAM_TILT_RAD
        body_x =  cz * math.cos(-tilt) - cy * math.sin(-tilt)
        body_y = -cx
        body_x += 0.20

        with self._lock:
            yaw = self._yaw
            rx  = self._x
            ry  = self._y

        gx = rx + body_x * math.cos(yaw) - body_y * math.sin(yaw)
        gy = ry + body_x * math.sin(yaw) + body_y * math.cos(yaw)

        print(f'[GOAL] body=({body_x:.3f}, {body_y:.3f})  '
              f'world=({gx:.3f}, {gy:.3f})  '
              f'rover=({rx:.3f}, {ry:.3f})  yaw={math.degrees(yaw):.1f}°', flush=True)

        with self._lock:
            self._goal_world = (gx, gy)
            self._path       = []
            self._path_idx   = 0
            self._state      = 'NAVIGATING'
            self._active     = True
            self._last_progress_time = time.monotonic()
            self._last_progress_pos  = (rx, ry)

        print(f'[GOAL] state → NAVIGATING  active=True  '
              f'waiting for replan in ≤{1/REPLAN_HZ:.1f}s', flush=True)
        self._publish_status('NAVIGATING')

    # ── Cancel ────────────────────────────────────────────────────────────

    def _cancel_cb(self, msg: Bool):
        if msg.data:
            with self._lock:
                self._active = False
                self._state  = 'IDLE'
                self._path   = []
            self._stop_motors()
            self.get_logger().info('Navigation cancelled')
            self._publish_status('IDLE')

    # ── Odometry from depth-flow node ────────────────────────────────────

    def _odom_cb(self, msg):
        """Accept pose from nav_depth_odom when available."""
        with self._lock:
            self._x   = msg.pose.pose.position.x
            self._y   = msg.pose.pose.position.y
            # Extract yaw from quaternion
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self._yaw = 2.0 * math.atan2(qz, qw)
            self._odom_available = True

    # ── Joystick active signal ────────────────────────────────────────────

    def _joy_active_cb(self, msg: Bool):
        # When joystick becomes active, pause dead reckoning update
        # (the mux already suppresses our cmd_vel, but we pause DR too)
        if msg.data:
            self._last_dr_time = self.get_clock().now()

    # ── Manual obstacle hints from GUI ────────────────────────────────────

    def _manual_obstacles_cb(self, msg: Float32MultiArray):
        """
        Receive manual obstacle 3D positions in CAMERA FRAME from GUI.
        Format: [cam_x, cam_y, cam_z,  cam_x, cam_y, cam_z, ...]

        We store them directly in camera frame. Each replan they are
        converted to rover-local grid coords fresh — no world-frame
        conversion so no drift from dead-reckoning errors.

        Empty array clears all manual obstacles.
        """
        data = list(msg.data)
        obstacles = []
        for i in range(0, len(data) - 2, 3):
            obstacles.append((float(data[i]),
                              float(data[i+1]),
                              float(data[i+2])))

        with self._lock:
            self._manual_obstacle_cam = obstacles   # camera frame (cx, cy, cz)

        self.get_logger().info(
            f'Manual obstacles: {len(obstacles)} points (camera frame)',
            throttle_duration_sec=1.0)

    # ── Dead reckoning ────────────────────────────────────────────────────

    def _dr_update(self):
        """Integrate cmd_vel for pose estimate — only used if depth odom unavailable."""
        if self._odom_available:
            return   # depth odom node handles pose; skip cmd_vel DR
        now = self.get_clock().now()
        dt  = (now - self._last_dr_time).nanoseconds / 1e9
        self._last_dr_time = now

        if dt <= 0 or dt > 0.5:
            return

        with self._lock:
            # These are set by _control_cb before publishing
            lin = getattr(self, '_last_lin', 0.0)
            ang = getattr(self, '_last_ang', 0.0)
            self._yaw += ang * dt
            self._yaw  = wrap_angle(self._yaw)
            self._x   += lin * math.cos(self._yaw) * dt
            self._y   += lin * math.sin(self._yaw) * dt

    # ── Occupancy grid from depth ─────────────────────────────────────────

    def _build_occupancy_grid(self) -> np.ndarray | None:
        """
        Build a GRID_CELLS × GRID_CELLS occupancy grid in the robot's
        local frame (robot at centre, x forward, y left).

        Returns numpy array with 0=free, 100=occupied, or None if no depth.
        """
        with self._depth_lock:
            if self._depth_img is None:
                return None
            depth = self._depth_img.copy()

        if self._fx is None:
            # Fall back to D435 424x240 approximate intrinsics
            self._fx = 213.0
            self._fy = 213.0
            self._cx = 212.0
            self._cy = 120.0

        h, w = depth.shape
        grid = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.int8)

        # Robot sits at grid centre
        origin = GRID_CELLS // 2   # row = origin → x forward, col = origin → y=0

        # Vectorised projection: process all pixels at once
        rows_idx, cols_idx = np.mgrid[0:h, 0:w]
        z_m = depth[rows_idx, cols_idx]   # distance along optical axis

        # Mask: valid depth, not in blind zone, not too far
        valid = (z_m > BLIND_ZONE_M) & (z_m < GRID_RANGE_M * 1.2) & (z_m > 0)

        z_v   = z_m[valid]
        row_v = rows_idx[valid]
        col_v = cols_idx[valid]

        # Back-project to camera 3D (camera frame: x=right, y=down, z=forward)
        cam_x = (col_v - self._cx) / self._fx * z_v
        cam_y = (row_v - self._cy) / self._fy * z_v
        cam_z = z_v

        # Rotate from camera frame to world-vertical frame
        # Camera is tilted tilt_rad around horizontal axis (x-axis)
        tilt = CAM_TILT_RAD   # negative
        # World y = cam_z * sin(tilt) + cam_y * cos(tilt)  (height above ground)
        # World x_fwd = cam_z * cos(tilt) - cam_y * sin(tilt)
        world_height = cam_z * math.sin(tilt) + cam_y * math.cos(tilt) + CAM_HEIGHT_M
        world_fwd    = cam_z * math.cos(tilt) - cam_y * math.sin(tilt)
        world_left   = -cam_x   # right in camera = left negative

        # Keep only pixels in the rock-height band
        rock_mask = (world_height > ROCK_HEIGHT_MIN_M) & (world_height < ROCK_HEIGHT_MAX_M)
        fwd_v  = world_fwd[rock_mask]
        left_v = world_left[rock_mask]

        # Map to grid cells
        # grid row 0 = furthest forward, row origin = robot, row -1 = behind
        grid_row = origin - (fwd_v  / GRID_RES_M).astype(int)
        grid_col = origin + (left_v / GRID_RES_M).astype(int)

        # Clamp and mark
        mask2 = (grid_row >= 0) & (grid_row < GRID_CELLS) & \
                (grid_col >= 0) & (grid_col < GRID_CELLS)
        grid[grid_row[mask2], grid_col[mask2]] = 100

        # Mark blind zone directly in front as free
        blind_cells = int(BLIND_ZONE_M / GRID_RES_M)
        blind_r0 = max(0, origin - blind_cells)
        grid[blind_r0:origin+1, :] = 0

        # Inflate obstacles
        if self._safety_cells > 0:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (self._safety_cells*2+1, self._safety_cells*2+1))
            inflated = cv2.dilate((grid > 50).astype(np.uint8), kernel)
            grid[inflated > 0] = 100
            # Restore blind zone even after inflation
            grid[blind_r0:origin+1, :] = 0

        # ── Inject manual obstacle hints from GUI ─────────────────────────
        # Manual obstacles arrive in camera frame (cam_x, cam_y, cam_z).
        # Convert to rover-body frame using the camera tilt, then to grid.
        # This avoids any world-frame / dead-reckoning drift.
        with self._lock:
            manual_cam = list(self._manual_obstacle_cam)

        for (cam_x, cam_y, cam_z) in manual_cam:
            # Camera → rover body (same rotation as _goal_cb)
            tilt   = CAM_TILT_RAD
            body_x =  cam_z * math.cos(-tilt) - cam_y * math.sin(-tilt)
            body_y = -cam_x

            # Body → grid (robot is always at grid origin)
            gr = origin - int(body_x / GRID_RES_M)
            gc = origin + int(body_y / GRID_RES_M)
            if 0 <= gr < GRID_CELLS and 0 <= gc < GRID_CELLS:
                r0 = max(0, gr - 3); r1 = min(GRID_CELLS, gr + 4)
                c0 = max(0, gc - 3); c1 = min(GRID_CELLS, gc + 4)
                grid[r0:r1, c0:c1] = 100

        return grid

    # ── A* path planning ──────────────────────────────────────────────────

    def _replan_cb(self):
        with self._lock:
            active = self._active
            state  = self._state
            gw     = self._goal_world
            rx, ry, ryaw = self._x, self._y, self._yaw

        # ── Guard: only run when navigating ───────────────────────────────
        if not active or state not in ('NAVIGATING', 'STUCK'):
            return   # silent — expected idle state

        if gw is None:
            print('[PLAN] ✗  no goal set', flush=True)
            return

        gx, gy = gw

        # ── Build occupancy grid ───────────────────────────────────────────
        grid = self._build_occupancy_grid()
        if grid is None:
            print('[PLAN] ✗  no depth image yet — cannot build grid', flush=True)
            return

        n_occupied = int((grid >= 50).sum())
        print(f'[PLAN] grid built  occupied_cells={n_occupied}/'
              f'{GRID_CELLS*GRID_CELLS}  '
              f'rover=({rx:.2f},{ry:.2f}) yaw={math.degrees(ryaw):.1f}°  '
              f'goal=({gx:.2f},{gy:.2f})', flush=True)

        self._publish_grid(grid)

        origin    = GRID_CELLS // 2
        start_rc  = (origin, origin)

        # Goal in robot-local frame
        dx_w = gx - rx;  dy_w = gy - ry
        dx_r =  dx_w * math.cos(-ryaw) - dy_w * math.sin(-ryaw)
        dy_r =  dx_w * math.sin(-ryaw) + dy_w * math.cos(-ryaw)
        dist_to_goal = math.sqrt(dx_w**2 + dy_w**2)

        print(f'[PLAN] goal local=({dx_r:.2f}fwd, {dy_r:.2f}left)  '
              f'dist={dist_to_goal:.2f}m', flush=True)

        if dist_to_goal < self._goal_tol:
            print(f'[PLAN] ✓  GOAL REACHED  dist={dist_to_goal:.2f}m '
                  f'< tol={self._goal_tol}m', flush=True)
            with self._lock:
                self._active = False
                self._state  = 'IDLE'
            self._stop_motors()
            self._publish_status('GOAL_REACHED')
            return

        goal_r = int(np.clip(origin - int(dx_r / GRID_RES_M), 1, GRID_CELLS-2))
        goal_c = int(np.clip(origin + int(dy_r / GRID_RES_M), 1, GRID_CELLS-2))
        goal_rc = (goal_r, goal_c)

        goal_cell_occ = grid[goal_r, goal_c] >= 50
        print(f'[PLAN] goal_cell=({goal_r},{goal_c})  '
              f'start_cell=({origin},{origin})  '
              f'goal_occupied={goal_cell_occ}', flush=True)

        if goal_cell_occ:
            goal_rc = self._nearest_free(grid, goal_r, goal_c)
            if goal_rc is None:
                print('[PLAN] ✗  goal blocked, no free cell nearby', flush=True)
                return
            print(f'[PLAN] goal nudged to ({goal_rc[0]},{goal_rc[1]})', flush=True)

        # ── A* ─────────────────────────────────────────────────────────────
        print(f'[PLAN] running A*  {start_rc} → {goal_rc}  '
              f'grid={GRID_CELLS}x{GRID_CELLS}', flush=True)
        path_rc = astar(grid, start_rc, goal_rc)

        if path_rc is None:
            print('[PLAN] ✗  A* found no path  '
                  f'(grid may be fully blocked, n_occ={n_occupied})', flush=True)
            with self._lock:
                self._state = 'STUCK'
            return

        print(f'[PLAN] ✓  A* path  {len(path_rc)} cells', flush=True)

        # Convert path from local grid → world coordinates
        path_world = []
        for (pr, pc) in path_rc:
            lx = (origin - pr) * GRID_RES_M
            ly = (pc - origin) * GRID_RES_M
            wx = rx + lx * math.cos(ryaw) - ly * math.sin(ryaw)
            wy = ry + lx * math.sin(ryaw) + ly * math.cos(ryaw)
            path_world.append((wx, wy))

        with self._lock:
            self._path     = path_world
            self._path_idx = 0
            if self._state == 'STUCK':
                self._state = 'NAVIGATING'

        # ── Print human-readable turn-by-turn directions ───────────────────
        print('[PLAN] ── DIRECTIONS ──────────────────────────', flush=True)
        seg_dist = 0.0
        prev_bearing = None
        for i in range(len(path_world)):
            wx, wy = path_world[i]
            lx =  (wx - rx) * math.cos(-ryaw) - (wy - ry) * math.sin(-ryaw)
            ly =  (wx - rx) * math.sin(-ryaw) + (wy - ry) * math.cos(-ryaw)

            if i + 1 < len(path_world):
                nx, ny = path_world[i + 1]
                bearing = math.atan2(ny - wy, nx - wx)
                seg = math.sqrt((nx-wx)**2 + (ny-wy)**2)
                seg_dist += seg

                if prev_bearing is None or abs(wrap_angle(bearing - prev_bearing)) > math.radians(15):
                    # Direction change — print a step
                    turn = math.degrees(wrap_angle(bearing - ryaw))
                    if   turn >  15: act = f'TURN LEFT  {turn:+.0f}°'
                    elif turn < -15: act = f'TURN RIGHT {turn:+.0f}°'
                    else:            act = 'STRAIGHT'
                    print(f'  step {i:3d}  local=({lx:+.2f}fwd {ly:+.2f}left)  '
                          f'{act}  then drive {seg_dist:.2f}m', flush=True)
                    seg_dist = 0.0
                    prev_bearing = bearing

        # Final waypoint
        if path_world:
            fx_l = (path_world[-1][0]-rx)*math.cos(-ryaw) - (path_world[-1][1]-ry)*math.sin(-ryaw)
            fy_l = (path_world[-1][0]-rx)*math.sin(-ryaw) + (path_world[-1][1]-ry)*math.cos(-ryaw)
            print(f'  GOAL  local=({fx_l:+.2f}fwd {fy_l:+.2f}left)  '
                  f'world=({path_world[-1][0]:.2f},{path_world[-1][1]:.2f})', flush=True)
        print(f'[PLAN] total path length ≈ '
              f'{sum(math.sqrt((path_world[i][0]-path_world[i-1][0])**2+(path_world[i][1]-path_world[i-1][1])**2) for i in range(1,len(path_world))):.2f}m',
              flush=True)
        print('[PLAN] ────────────────────────────────────────', flush=True)

        self._publish_path(path_world)

    # ── Control loop ──────────────────────────────────────────────────────

    def _control_cb(self):
        with self._lock:
            active = self._active
            state  = self._state
            path   = list(self._path)
            pidx   = self._path_idx
            rx, ry, ryaw = self._x, self._y, self._yaw
            gx, gy = self._goal_world if self._goal_world else (0, 0)

        if not active:
            return

        # ── Goal reached ─────────────────────────────────────────────────
        dist_goal = math.sqrt((gx-rx)**2 + (gy-ry)**2)
        if dist_goal < self._goal_tol:
            with self._lock:
                self._active = False
                self._state  = 'IDLE'
            self._stop_motors()
            self.get_logger().info(f'Goal reached! dist={dist_goal:.2f}m')
            self._publish_status('GOAL_REACHED')
            return

        # ── Stuck escape: slow pivot ──────────────────────────────────────
        if state == 'STUCK':
            now = time.monotonic()
            if not hasattr(self, '_pivot_start') or self._pivot_start == 0:
                self._pivot_start = now
                self._pivot_dir   = 1.0  # try left first
                self.get_logger().warn('Stuck — attempting escape pivot')

            if now - self._pivot_start < PIVOT_DURATION_S:
                self._send_vel(0.0, PIVOT_SPEED * self._pivot_dir)
            else:
                self._pivot_start = 0
                with self._lock:
                    self._state = 'NAVIGATING'
            return

        # ── No path yet ───────────────────────────────────────────────────
        if not path:
            print('[CTRL] active but no path yet — waiting for replan', flush=True)
            return

        # ── Advance path index ────────────────────────────────────────────
        while pidx < len(path) - 1:
            wx, wy = path[pidx]
            d = math.sqrt((wx-rx)**2 + (wy-ry)**2)
            if d < WAYPOINT_TOL_M:
                pidx += 1
            else:
                break

        with self._lock:
            self._path_idx = pidx

        # ── Pure pursuit ──────────────────────────────────────────────────
        # Find the lookahead point on the path
        target = None
        for i in range(pidx, len(path)):
            wx, wy = path[i]
            d = math.sqrt((wx-rx)**2 + (wy-ry)**2)
            if d >= LOOKAHEAD_M:
                target = (wx, wy)
                break

        if target is None:
            target = path[-1]   # use final point

        tx, ty = target
        dx = tx - rx
        dy = ty - ry

        # Desired heading
        desired_yaw = math.atan2(dy, dx)
        yaw_err     = wrap_angle(desired_yaw - ryaw)

        dist_to_target = math.sqrt(dx**2 + dy**2)

        # ── Stuck detection ───────────────────────────────────────────────
        now = time.monotonic()
        moved = math.sqrt((rx - self._last_progress_pos[0])**2 +
                          (ry - self._last_progress_pos[1])**2)
        if moved > STUCK_DIST_M:
            self._last_progress_time = now
            self._last_progress_pos  = (rx, ry)
        elif now - self._last_progress_time > STUCK_TIME_S:
            with self._lock:
                self._state = 'STUCK'
            self.get_logger().warn('Stuck detected — triggering escape')
            return

        # ── Arc turn control ──────────────────────────────────────────────
        # Scale forward speed based on heading error
        # Large heading error → slow down and turn
        # Small heading error → drive forward with slight correction
        heading_factor = max(0.0, math.cos(yaw_err))   # 1.0 straight, 0.0 sideways
        lin = self._lin_speed * heading_factor

        # Slow down when approaching goal
        if dist_to_target < 1.0:
            lin *= (dist_to_target / 1.0)
            lin  = max(lin, 0.08)   # minimum creep speed

        ang = float(np.clip(yaw_err * ARC_BLEND * self._ang_speed,
                            -self._ang_speed, self._ang_speed))

        # Ensure we're always moving (arc, not pivot)
        if abs(yaw_err) > math.radians(60):
            lin = max(lin, 0.05)

        self._ctrl_print_n += 1
        if self._ctrl_print_n % 10 == 1:   # print ~1 Hz (every 10th at 10 Hz)
            print(f'[CTRL] target=({tx:.2f},{ty:.2f})  '
                  f'yaw_err={math.degrees(yaw_err):+.1f}°  '
                  f'lin={lin:.3f}m/s  ang={ang:+.3f}rad/s  '
                  f'waypoint={pidx}/{len(path)}  '
                  f'dist_goal={dist_goal:.2f}m', flush=True)

        self._send_vel(lin, ang)

    # ── TF broadcast + RViz visualisation (20 Hz) ────────────────────────

    def _tf_viz_cb(self):
        if not hasattr(self, '_viz_tick'): self._viz_tick = 0
        self._viz_tick += 1
        if self._viz_tick % 20 == 1:
            print(f'[VIZ_TICK] #{self._viz_tick}', flush=True)
        try:
            self._tf_viz_body()
        except Exception as exc:
            import traceback
            print(f'[VIZ] *** CRASH: {exc}', flush=True)
            traceback.print_exc()

    def _tf_viz_body(self):
        with self._lock:
            rx, ry, ryaw = self._x, self._y, self._yaw
            path   = list(self._path)
            pidx   = self._path_idx
            gw     = self._goal_world
            active = self._active

        stamp = self.get_clock().now().to_msg()

        # ── 1. Broadcast TF odom → base_link ─────────────────────────────
        if self._tf_br is not None:
            tf       = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_link'
            tf.transform.translation.x = rx
            tf.transform.translation.y = ry
            tf.transform.translation.z = 0.0
            tf.transform.rotation.z    = math.sin(ryaw / 2)
            tf.transform.rotation.w    = math.cos(ryaw / 2)
            self._tf_br.sendTransform(tf)

        # ── 2. Build viz markers ──────────────────────────────────────────
        ma     = MarkerArray()
        mid    = 0

        def new_marker(ns, typ, frame='odom'):
            nonlocal mid
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = frame
            m.ns     = ns
            m.id     = mid
            m.type   = typ
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime.sec  = 1    # auto-clear after 1s if we stop publishing
            m.lifetime.nanosec = 0
            mid += 1
            return m

        # ── Rover footprint ───────────────────────────────────────────────
        rp = new_marker('rover', Marker.CYLINDER)
        rp.pose.position.x  = rx
        rp.pose.position.y  = ry
        rp.pose.position.z  = 0.05
        rp.pose.orientation.z = math.sin(ryaw / 2)
        rp.pose.orientation.w = math.cos(ryaw / 2)
        rp.scale.x = 0.40   # rover diameter
        rp.scale.y = 0.40
        rp.scale.z = 0.10
        rp.color   = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.4)
        ma.markers.append(rp)

        # Heading arrow on rover
        fwd = new_marker('rover', Marker.ARROW)
        fwd.pose.position.x  = rx
        fwd.pose.position.y  = ry
        fwd.pose.position.z  = 0.12
        fwd.pose.orientation.z = math.sin(ryaw / 2)
        fwd.pose.orientation.w = math.cos(ryaw / 2)
        fwd.scale.x = 0.50   # shaft length
        fwd.scale.y = 0.06   # shaft diameter
        fwd.scale.z = 0.06
        fwd.color   = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.9)
        ma.markers.append(fwd)

        if active and path:
            # ── Path line strip (remaining) ───────────────────────────────
            line = new_marker('path', Marker.LINE_STRIP)
            line.scale.x = 0.04   # line width metres
            line.color   = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)

            # Start from rover position
            line.points.append(Point(x=rx, y=ry, z=0.03))
            for i in range(pidx, len(path)):
                wx, wy = path[i]
                line.points.append(Point(x=wx, y=wy, z=0.03))
            if len(line.points) >= 2:
                ma.markers.append(line)

            # ── Done segments (grey) ──────────────────────────────────────
            if pidx > 0:
                done_line = new_marker('path_done', Marker.LINE_STRIP)
                done_line.scale.x = 0.02
                done_line.color   = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.5)
                for i in range(min(pidx + 1, len(path))):
                    wx, wy = path[i]
                    done_line.points.append(Point(x=wx, y=wy, z=0.02))
                if len(done_line.points) >= 2:
                    ma.markers.append(done_line)

            # ── Distance labels every 0.5 m along remaining path ─────────
            cum  = 0.0
            prev = (rx, ry)
            next_label_m = 0.5
            for i in range(pidx, len(path)):
                wx, wy = path[i]
                seg = math.sqrt((wx-prev[0])**2 + (wy-prev[1])**2)
                cum += seg
                while cum >= next_label_m:
                    # Interpolate position at next_label_m
                    frac = 1.0 - (cum - next_label_m) / max(seg, 1e-6)
                    frac = max(0.0, min(1.0, frac))
                    lx   = prev[0] + frac * (wx - prev[0])
                    ly   = prev[1] + frac * (wy - prev[1])

                    # Tick sphere
                    tk = new_marker('dist_ticks', Marker.SPHERE)
                    tk.pose.position.x = lx
                    tk.pose.position.y = ly
                    tk.pose.position.z = 0.05
                    tk.scale.x = tk.scale.y = tk.scale.z = 0.08
                    tk.color   = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)
                    ma.markers.append(tk)

                    # Distance text
                    txt = new_marker('dist_labels', Marker.TEXT_VIEW_FACING)
                    txt.pose.position.x = lx
                    txt.pose.position.y = ly
                    txt.pose.position.z = 0.22
                    txt.scale.z = 0.18   # text height metres
                    txt.color   = ColorRGBA(r=1.0, g=1.0, b=0.3, a=1.0)
                    txt.text    = f'{next_label_m:.1f}m'
                    ma.markers.append(txt)

                    next_label_m += 0.5
                prev = (wx, wy)

            # ── Turn arrows at significant heading changes ─────────────────
            prev_bearing = None
            for i in range(pidx, len(path) - 1):
                ax, ay = path[i]
                bx, by = path[i + 1]
                bearing = math.atan2(by - ay, bx - ax)
                if prev_bearing is not None:
                    delta = abs(wrap_angle(bearing - prev_bearing))
                    if delta > math.radians(20):
                        arr = new_marker('turn_arrows', Marker.ARROW)
                        arr.pose.position.x = ax
                        arr.pose.position.y = ay
                        arr.pose.position.z = 0.15
                        arr.pose.orientation.z = math.sin(bearing / 2)
                        arr.pose.orientation.w = math.cos(bearing / 2)
                        arr.scale.x = 0.35   # shaft length
                        arr.scale.y = 0.08
                        arr.scale.z = 0.08
                        deg = math.degrees(wrap_angle(bearing - ryaw))
                        # Colour: left=cyan, right=orange
                        if deg > 0:
                            arr.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)
                        else:
                            arr.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
                        ma.markers.append(arr)

                        # Turn label
                        tlbl = new_marker('turn_labels', Marker.TEXT_VIEW_FACING)
                        tlbl.pose.position.x = ax
                        tlbl.pose.position.y = ay
                        tlbl.pose.position.z = 0.35
                        tlbl.scale.z = 0.15
                        tlbl.color   = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
                        sym   = '◄' if deg > 0 else '►'
                        tlbl.text = f'{sym}{abs(deg):.0f}°'
                        ma.markers.append(tlbl)

                prev_bearing = bearing

            # ── Next-waypoint arrow from rover ────────────────────────────
            if pidx < len(path):
                twx, twy = path[pidx]
                bearing_to_wp = math.atan2(twy - ry, twx - rx)
                nw = new_marker('next_wp', Marker.ARROW)
                nw.pose.position.x  = rx
                nw.pose.position.y  = ry
                nw.pose.position.z  = 0.20
                nw.pose.orientation.z = math.sin(bearing_to_wp / 2)
                nw.pose.orientation.w = math.cos(bearing_to_wp / 2)
                nw.scale.x = min(0.60, math.sqrt((twx-rx)**2+(twy-ry)**2) * 0.8)
                nw.scale.y = 0.10
                nw.scale.z = 0.10
                nw.color   = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # yellow
                ma.markers.append(nw)

                # Distance + action text above rover
                dist_wp = math.sqrt((twx-rx)**2+(twy-ry)**2)
                yaw_err = math.degrees(wrap_angle(bearing_to_wp - ryaw))
                if   yaw_err >  15: act_str = f'◄ {yaw_err:+.0f}°'
                elif yaw_err < -15: act_str = f'► {yaw_err:+.0f}°'
                else:               act_str = f'▲ fwd'
                action_txt = new_marker('action', Marker.TEXT_VIEW_FACING)
                action_txt.pose.position.x = rx
                action_txt.pose.position.y = ry
                action_txt.pose.position.z = 0.55
                action_txt.scale.z = 0.20
                action_txt.color   = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                action_txt.text    = f'{act_str}  {dist_wp:.1f}m'
                ma.markers.append(action_txt)

            # ── Distance remaining label near goal ────────────────────────
            if gw:
                dist_goal = math.sqrt((gw[0]-rx)**2+(gw[1]-ry)**2)
                dr_len = sum(
                    math.sqrt((path[i][0]-path[i-1][0])**2+(path[i][1]-path[i-1][1])**2)
                    for i in range(max(1, pidx), len(path))
                ) if len(path) > 1 else 0.0

                remain_txt = new_marker('remaining', Marker.TEXT_VIEW_FACING)
                remain_txt.pose.position.x = gw[0]
                remain_txt.pose.position.y = gw[1]
                remain_txt.pose.position.z = 0.60
                remain_txt.scale.z = 0.22
                remain_txt.color   = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
                remain_txt.text    = f'GOAL\n{dr_len:.1f}m remaining'
                ma.markers.append(remain_txt)

        # ── Goal sphere ───────────────────────────────────────────────────
        if gw and active:
            gs = new_marker('goal', Marker.SPHERE)
            gs.pose.position.x = gw[0]
            gs.pose.position.y = gw[1]
            gs.pose.position.z = 0.15
            gs.scale.x = gs.scale.y = gs.scale.z = 0.30
            gs.color   = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.8)
            ma.markers.append(gs)

            # Pulsing ring (flat cylinder)
            ring = new_marker('goal_ring', Marker.CYLINDER)
            ring.pose.position.x = gw[0]
            ring.pose.position.y = gw[1]
            ring.pose.position.z = 0.01
            ring.scale.x = ring.scale.y = 0.60
            ring.scale.z = 0.02
            ring.color   = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.3)
            ma.markers.append(ring)

        # ── Delete stale markers if idle ──────────────────────────────────
        # (lifetime=1s handles this automatically, nothing extra needed)

        self._viz_pub.publish(ma)
        if self._viz_tick % 20 == 1:
            print(f'[VIZ] OK markers={len(ma.markers)} active={active} path={len(path)}', flush=True)

    # ── Path progress publisher ───────────────────────────────────────────

    def _progress_cb(self):
        """
        Publish a JSON string on /nav/path_progress at 5 Hz describing:
          - Full path in current robot-local coords (fwd, left per waypoint)
          - Current waypoint index
          - Distance remaining to goal
          - Immediate action: turn angle + straight distance to next waypoint
          - Cumulative distance markers every 0.5m
          - Whether the rover is on-track or deviating

        The GUI uses this to draw an annotated path overlay that updates
        live as the rover moves without waiting for a full replan.
        """
        import json as _json

        with self._lock:
            path   = list(self._path)
            pidx   = self._path_idx
            rx, ry, ryaw = self._x, self._y, self._yaw
            state  = self._state
            active = self._active
            gx, gy = self._goal_world if self._goal_world else (None, None)

        if not path or gx is None:
            m = String()
            m.data = _json.dumps({'state': state, 'active': active,
                                  'waypoints': [], 'progress': {}})
            self._progress_pub.publish(m)
            return

        # Convert remaining path waypoints to robot-local frame
        waypoints = []
        cum_dist  = 0.0
        prev_wx, prev_wy = rx, ry

        for i, (wx, wy) in enumerate(path):
            dx_w = wx - rx
            dy_w = wy - ry
            # Robot-local (fwd = x, left = y)
            lx =  dx_w * math.cos(-ryaw) - dy_w * math.sin(-ryaw)
            ly =  dx_w * math.sin(-ryaw) + dy_w * math.cos(-ryaw)

            seg = math.sqrt((wx - prev_wx)**2 + (wy - prev_wy)**2)
            cum_dist += seg
            prev_wx, prev_wy = wx, wy

            waypoints.append({
                'idx':     i,
                'fwd':     round(lx, 3),
                'left':    round(ly, 3),
                'cum_m':   round(cum_dist, 3),
                'current': (i == pidx),
                'done':    (i < pidx),
            })

        # Distance remaining: sum of segments from current idx to end
        dist_remaining = 0.0
        prx, pry = rx, ry
        for wx, wy in path[pidx:]:
            dist_remaining += math.sqrt((wx-prx)**2 + (wy-pry)**2)
            prx, pry = wx, wy

        # Distance from rover to goal directly
        dist_to_goal = math.sqrt((gx-rx)**2 + (gy-ry)**2) if gx else 0.0

        # Immediate action: what should the rover do RIGHT NOW
        action = {}
        if pidx < len(path):
            twx, twy = path[min(pidx, len(path)-1)]
            tdx =  (twx - rx) * math.cos(-ryaw) - (twy - ry) * math.sin(-ryaw)
            tdy =  (twx - rx) * math.sin(-ryaw) + (twy - ry) * math.cos(-ryaw)
            turn_rad   = math.atan2(tdy, tdx)
            turn_deg   = math.degrees(turn_rad)
            seg_dist   = math.sqrt(tdx**2 + tdy**2)

            if   turn_deg >  15: direction = 'TURN LEFT'
            elif turn_deg < -15: direction = 'TURN RIGHT'
            else:                direction = 'DRIVE STRAIGHT'

            action = {
                'direction':  direction,
                'turn_deg':   round(turn_deg, 1),
                'seg_dist_m': round(seg_dist, 2),
            }

        # Cross-track error: perpendicular distance from rover to current path segment
        cte = 0.0
        if pidx > 0 and pidx < len(path):
            ax, ay = path[pidx - 1]
            bx, by = path[pidx]
            abx, aby = bx - ax, by - ay
            ab_len = math.sqrt(abx**2 + aby**2) or 1e-6
            # Project rover position onto segment
            t = ((rx - ax) * abx + (ry - ay) * aby) / (ab_len**2)
            t = max(0.0, min(1.0, t))
            closest_x = ax + t * abx
            closest_y = ay + t * aby
            cte = math.sqrt((rx - closest_x)**2 + (ry - closest_y)**2)

        progress = {
            'dist_remaining_m': round(dist_remaining, 2),
            'dist_to_goal_m':   round(dist_to_goal,   2),
            'waypoint_idx':     pidx,
            'total_waypoints':  len(path),
            'cross_track_err_m': round(cte, 3),
            'on_track':         cte < 0.25,
            'state':            state,
            'action':           action,
        }

        m = String()
        m.data = _json.dumps({
            'state':     state,
            'active':    active,
            'waypoints': waypoints,
            'progress':  progress,
        })
        self._progress_pub.publish(m)

        # Print summary every 5 publishes (~1 Hz)
        if not hasattr(self, '_prog_print_n'): self._prog_print_n = 0
        self._prog_print_n += 1
        if self._prog_print_n % 5 == 1:
            act = progress.get('action', {})
            print(f'[PROG] state={state}  '
                  f'remaining={progress.get("dist_remaining_m","?"):.2f}m  '
                  f'waypt={progress.get("waypoint_idx","?")}/'
                  f'{progress.get("total_waypoints","?")}  '
                  f'CTE={progress.get("cross_track_err_m","?"):.3f}m  '
                  f'action={act.get("direction","—")} '
                  f'{act.get("turn_deg",0):.0f}° '
                  f'{act.get("seg_dist_m",0):.2f}m', flush=True)

    # ── Motor helpers ─────────────────────────────────────────────────────

    def _send_vel(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(np.clip(lin, -self._lin_speed, self._lin_speed))
        t.angular.z = float(np.clip(ang, -self._ang_speed, self._ang_speed))
        self._last_lin = t.linear.x
        self._last_ang = t.angular.z
        self._cmd_pub.publish(t)

    def _stop_motors(self):
        self._send_vel(0.0, 0.0)
        self._last_lin = 0.0
        self._last_ang = 0.0

    # ── Publishing helpers ────────────────────────────────────────────────

    def _publish_path(self, path_world: list):
        """
        Publish path in ODOM frame (world coordinates).
        RViz fixed frame is 'odom', so these stay in place correctly.
        The TF broadcaster keeps base_link moving relative to odom.
        """
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        for (wx, wy) in path_world:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.position.z = 0.03
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self._path_pub.publish(msg)
        print(f'[PATH] published {len(msg.poses)} poses  frame=odom', flush=True)

    def _publish_grid(self, grid: np.ndarray):
        """
        Publish occupied cells as MarkerArray cubes in ODOM frame.
        Grid cells are in robot-local coords; we rotate them into world/odom
        coords so the markers stay fixed in the RViz 3D view as the rover moves.
        """
        with self._lock:
            rx, ry, ryaw = self._x, self._y, self._yaw

        ma    = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        origin = GRID_CELLS // 2

        clear = Marker()
        clear.header.stamp    = stamp
        clear.header.frame_id = 'odom'
        clear.ns     = 'obstacles'
        clear.id     = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        mid = 1
        occ_rows, occ_cols = np.where(grid >= 50)
        for pr, pc in zip(occ_rows, occ_cols):
            # Local frame (robot-centric)
            lx = (origin - pr) * GRID_RES_M   # forward
            ly = (pc - origin) * GRID_RES_M   # left

            # Rotate to odom/world frame
            wx = rx + lx * math.cos(ryaw) - ly * math.sin(ryaw)
            wy = ry + lx * math.sin(ryaw) + ly * math.cos(ryaw)

            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = 'odom'
            m.ns     = 'obstacles'
            m.id     = mid
            m.type   = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = float(GRID_RES_M)
            m.scale.y = float(GRID_RES_M)
            m.scale.z = 0.30
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.0
            m.color.a = 0.6
            m.lifetime.sec = 1
            ma.markers.append(m)
            mid += 1

        self._grid_pub.publish(ma)
        print(f'[GRID] published {mid-1} obstacle cubes  frame=odom', flush=True)

    def _publish_status(self, status: str):
        m = String()
        m.data = status
        self._stat_pub.publish(m)

    # ── Utility ───────────────────────────────────────────────────────────

    def _nearest_free(self, grid, r, c, max_search=20):
        """BFS to find nearest free cell to (r,c)."""
        from collections import deque as _deque
        q = _deque([(r, c)])
        seen = {(r, c)}
        rows, cols = grid.shape
        while q:
            cr, cc = q.popleft()
            if grid[cr, cc] < 50:
                return (cr, cc)
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = cr+dr, cc+dc
                if (nr, nc) not in seen and 0<=nr<rows and 0<=nc<cols:
                    if abs(nr-r) + abs(nc-c) < max_search:
                        seen.add((nr, nc))
                        q.append((nr, nc))
        return None


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = NavDepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_motors()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()