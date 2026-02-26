#!/usr/bin/env python3
"""
slam_launch.py  —  Single-machine RTAB-Map SLAM launcher
=========================================================
Replaces slam_minipc.sh for parameter handling.
Python sets all parameters as typed objects — no shell quoting, no
ROS2 CLI type inference, no string: prefixes.

Usage:
  python3 slam_launch.py              # fresh map
  python3 slam_launch.py --keep       # resume existing map
  python3 slam_launch.py --localize   # localize against saved map

Then in a second terminal:  bash slam_laptop.sh
"""

import argparse
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

# ── Args ─────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--keep',     action='store_true', help='Resume existing map DB')
parser.add_argument('--localize', action='store_true', help='Localize-only mode')
args = parser.parse_args()

DB_PATH = os.path.expanduser('~/.ros/rtabmap_rover.db')
PROCS   = []   # track all launched subprocesses

def cleanup(sig=None, frame=None):
    print('\nShutting down all nodes...')
    for p in reversed(PROCS):
        try:
            p.terminate()
        except Exception:
            pass
    time.sleep(1)
    for p in PROCS:
        try:
            p.kill()
        except Exception:
            pass
    print('Done.')
    sys.exit(0)

signal.signal(signal.SIGINT,  cleanup)
signal.signal(signal.SIGTERM, cleanup)

def run(*cmd, log=None, check_delay=None):
    """Launch a subprocess, optionally logging to file."""
    env = os.environ.copy()
    stdout = open(log, 'w') if log else None
    stderr = open(log, 'a') if log else None
    p = subprocess.Popen(list(cmd), env=env, stdout=stdout, stderr=stderr)
    PROCS.append(p)
    if check_delay:
        time.sleep(check_delay)
        if p.poll() is not None:
            print(f'\n✗ Process died: {cmd[0]} {cmd[1]}')
            if log:
                print(f'  Last log lines ({log}):')
                try:
                    lines = open(log).readlines()
                    for l in lines[-20:]:
                        print('   ', l.rstrip())
                except Exception:
                    pass
            cleanup()
    return p

def wait_for_topic(topic, timeout=30):
    """Poll until a topic appears in ros2 topic list."""
    print(f'  Waiting for {topic}...', end='', flush=True)
    for _ in range(timeout):
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=5
        )
        if topic in result.stdout:
            print(' ✓')
            return True
        print('.', end='', flush=True)
        time.sleep(1)
    print(' ✗ (timeout)')
    return False

def wait_for_tf(source, target, timeout=20):
    """Poll TF tree by checking /tf topic for the source frame, with proper timeout handling."""
    print(f'  Waiting for TF {source}\u2192{target}...', end='', flush=True)
    for _ in range(timeout):
        # Use ros2 topic echo with --once and a short timeout via Popen+communicate
        # tf2_echo is a long-running node so we can't use subprocess.run on it safely.
        # Instead check if source frame appears in /tf_static or /tf broadcasts.
        try:
            proc = subprocess.Popen(
                ['ros2', 'topic', 'echo', '--once', '--no-daemon', '/tf_static'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            try:
                out, _ = proc.communicate(timeout=2)
                if source in out or target in out:
                    print(' ✓')
                    proc.kill()
                    return True
            except subprocess.TimeoutExpired:
                proc.kill()
        except Exception:
            pass
        # Also check via ros2 node list that robot_state_publisher is up
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5
            )
            if 'robot_state_publisher' in result.stdout:
                # RSP is up — give it 1 more second then proceed
                time.sleep(1)
                print(' ✓ (rsp confirmed)')
                return True
        except Exception:
            pass
        print('.', end='', flush=True)
        time.sleep(1)
    print(' (continuing without TF confirmation)')
    return False

# ── ROS env ───────────────────────────────────────────────────────────────
for ros_setup in ['/opt/ros/jazzy/setup.bash', '/opt/ros/humble/setup.bash']:
    if os.path.exists(ros_setup):
        ROS_DISTRO = 'jazzy' if 'jazzy' in ros_setup else 'humble'
        # Source by reading env from a subprocess
        result = subprocess.run(
            ['bash', '-c', f'source {ros_setup} && env'],
            capture_output=True, text=True
        )
        for line in result.stdout.splitlines():
            if '=' in line:
                k, _, v = line.partition('=')
                os.environ[k] = v
        break

ws_setup = os.path.expanduser('~/lunar_rover_ws/install/setup.bash')
if os.path.exists(ws_setup):
    result = subprocess.run(
        ['bash', '-c', f'source {ws_setup} && env'],
        capture_output=True, text=True
    )
    for line in result.stdout.splitlines():
        if '=' in line:
            k, _, v = line.partition('=')
            os.environ[k] = v

os.environ['ROS_DOMAIN_ID']                  = '42'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']  = 'SUBNET'
os.environ['ROS_LOCALHOST_ONLY']             = '0'

# ── FastDDS: disable SHM transport ───────────────────────────────────────
FASTDDS_XML = '/tmp/fastdds_udp_only.xml'
with open(FASTDDS_XML, 'w') as f:
    f.write('''<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UDPv4Transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UDPv4Transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>''')
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = FASTDDS_XML

# Clean stale SHM locks
for d in ['/dev/shm', '/tmp']:
    for f in os.listdir(d):
        if f.startswith('fastrtps_'):
            try:
                os.remove(os.path.join(d, f))
            except Exception:
                pass

# ── Header ────────────────────────────────────────────────────────────────
print('=========================================')
if args.localize:
    print('  SLAM — LOCALIZATION MODE')
elif args.keep:
    print('  SLAM — MAPPING (RESUME)')
else:
    print('  SLAM — MAPPING (FRESH)')
print('=========================================')
print(f'ROS2: {ROS_DISTRO}  |  DOMAIN: 42')
print(f'Map:  {DB_PATH}')
print()

if args.localize and not os.path.exists(DB_PATH):
    print(f'✗ No map at {DB_PATH} — run mapping first')
    sys.exit(1)

# ── [1/5] TF TREE ─────────────────────────────────────────────────────────
print('[1/5] TF tree + robot description...')

URDF = '''<?xml version="1.0"?>
<robot name="lunar_rover">
  <link name="base_link"/>
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <link name="camera_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>'''

run('ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
    '--ros-args',
    '-p', f'robot_description:={URDF}',
    '-p', 'publish_frequency:=50.0',
    log='/tmp/slam_rsp.log')

time.sleep(1)

# Static TFs: camera_link → optical frames (90° rotation)
for child in ['camera_depth_optical_frame', 'camera_color_optical_frame']:
    run('ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        '--x', '0', '--y', '0', '--z', '0',
        '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
        '--frame-id', 'camera_link',
        '--child-frame-id', child,
        log='/dev/null')

# Identity map→odom so RViz has a valid map frame before first keyframe
run('ros2', 'run', 'tf2_ros', 'static_transform_publisher',
    '--x', '0', '--y', '0', '--z', '0',
    '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
    '--frame-id', 'map',
    '--child-frame-id', 'odom',
    log='/dev/null')

wait_for_tf('base_link', 'camera_color_optical_frame', timeout=20)
print()

# ── [2/5] DEAD-RECKONING ODOMETRY ─────────────────────────────────────────
print('[2/5] Dead-reckoning odometry...')

ODOM_SCRIPT = '''\
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class DeadReckonOdom(Node):
    def __init__(self):
        super().__init__("dead_reckoning_odom")
        self.x = self.y = self.th = self.vx = self.vth = 0.0
        self.last = self.get_clock().now()
        self.br  = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "/odom", 50)
        self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.create_timer(0.02, self.update)
        self.get_logger().info("Dead-reckoning odometry running")

    def cb(self, msg):
        self.vx  = msg.linear.x
        self.vth = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt  = (now - self.last).nanoseconds / 1e9
        self.last = now
        if dt <= 0 or dt > 0.5:
            return
        self.x  += self.vx * math.cos(self.th) * dt
        self.y  += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt
        qz = math.sin(self.th / 2)
        qw = math.cos(self.th / 2)
        stamp = now.to_msg()
        t = TransformStamped()
        t.header.stamp       = stamp
        t.header.frame_id    = "odom"
        t.child_frame_id     = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.br.sendTransform(t)
        o = Odometry()
        o.header.stamp    = stamp
        o.header.frame_id = "odom"
        o.child_frame_id  = "base_footprint"
        o.pose.pose.position.x    = self.x
        o.pose.pose.position.y    = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x    = self.vx
        o.twist.twist.angular.z   = self.vth
        for i in [0, 7, 35]: o.pose.covariance[i]  = 0.05
        for i in [0, 35]:    o.twist.covariance[i] = 0.05
        self.pub.publish(o)

rclpy.init()
n = DeadReckonOdom()
try:    rclpy.spin(n)
except: pass
finally:
    n.destroy_node()
    rclpy.try_shutdown()
'''

odom_script_path = '/tmp/slam_odom_node.py'
with open(odom_script_path, 'w') as f:
    f.write(ODOM_SCRIPT)

run('python3', odom_script_path, log='/tmp/slam_odom.log', check_delay=2)
print('  ✓ Odometry publishing on /odom')
print()

# ── [3/5] CAMERA ─────────────────────────────────────────────────────────
print('[3/5] D435 Camera...')

run('ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
    'camera_name:=camera',
    'camera_namespace:=camera',
    'enable_depth:=true',
    'enable_color:=true',
    'enable_infra1:=false',
    'enable_infra2:=false',
    'pointcloud.enable:=true',
    'align_depth.enable:=true',
    'enable_sync:=true',
    'depth_module.profile:=640x480x15',
    'rgb_camera.profile:=640x480x15',
    log='/tmp/slam_camera.log')

if not wait_for_topic('/camera/camera/color/image_raw', timeout=30):
    print('  ✗ Camera failed — check: tail /tmp/slam_camera.log')
    cleanup()

run('ros2', 'run', 'depthimage_to_laserscan', 'depthimage_to_laserscan_node',
    '--ros-args',
    '--remap', 'depth/image_raw:=/camera/camera/aligned_depth_to_color/image_raw',
    '--remap', 'depth/camera_info:=/camera/camera/depth/camera_info',
    '--remap', 'scan:=/scan',
    '-p', 'scan_height:=1',
    '-p', 'range_min:=0.2',
    '-p', 'range_max:=5.0',
    '-p', 'output_frame_id:=camera_link',
    log='/tmp/slam_scan.log')

print('  ✓ Camera ready')
print()

# ── [4/5] VISUAL ODOMETRY ─────────────────────────────────────────────────
print('[4/5] RTAB-Map visual odometry...')

# Parameters are Python strings — no shell, no type inference, no quoting issues.
# ROS2 Python API receives them directly as strings, which is exactly what
# RTAB-Map expects for its Odom/*, Vis/* parameter family.
VODOM_YAML = '/tmp/vodom_params.yaml'
with open(VODOM_YAML, 'w') as yf:
    yf.write("""rgbd_odometry:
  ros__parameters:
    # Native ROS2 typed params
    frame_id: "base_link"
    odom_frame_id: "odom"
    publish_tf: true
    approx_sync: true
    approx_sync_max_interval: 0.2
    wait_for_transform: 0.5
    queue_size: 30
    # RTAB-Map internal params — string type in parameter server
    #
    # Odom/Strategy: "0" = Frame-to-Map (F2M) — robust, keeps a local map
    #   of features and matches new frames against it. Better than F2F (1)
    #   for slow handheld motion since it doesn't need consecutive frames
    #   to overlap perfectly.
    Odom/Strategy: "0"
    #
    # Odom/GuessMotion: "true" — use previous velocity to predict next pose.
    #   Helps when camera moves smoothly; reduces search space for matching.
    Odom/GuessMotion: "true"
    #
    # Odom/ResetCountdown: "0" — CRITICAL: disable auto-reset on tracking loss.
    #   Default "1" means one failed frame = full odometry reset = map wipe.
    #   "0" means keep trying to recover instead of resetting immediately.
    Odom/ResetCountdown: "0"
    #
    # Vis/FeatureType: "8" = SIFT — more robust to lighting/blur than ORB (6)
    #   or GFTT (0). Slower but much better for indoor handheld use.
    #   Fall back to "0" (GFTT+BRIEF) if too slow on this hardware.
    Vis/FeatureType: "6"
    #
    # Vis/MaxFeatures: "1000" — more features = more chances to find matches
    #   when the scene has low texture or partial occlusion.
    Vis/MaxFeatures: "500"
    #
    # Vis/MinInliers: "6" — lowered from 8. Minimum inliers to accept a
    #   frame. Lower = more tolerant of difficult scenes; higher = more
    #   accurate but resets more often. 6 is a good balance.
    Vis/MinInliers: "6"
    #
    # Vis/MaxDepth: "4.0" — ignore depth beyond 4m. D435 depth quality
    #   degrades past ~3-4m indoors; distant noisy points hurt matching.
    Vis/MaxDepth: "4.0"
    #
    # Vis/MinDepth: "0.3" — ignore depth closer than 30cm (D435 minimum
    #   reliable range). Eliminates noise from objects too close to camera.
    Vis/MinDepth: "0.3"
    #
    # Odom/FilteringStrategy: "1" — Kalman filter on odometry output.
    #   Smooths jitter between frames, especially important for slow motion.
    Odom/FilteringStrategy: "1"
    #
    # F2M local map size — keep last 2000 features in the local map for
    #   Frame-to-Map matching. Larger = more robust, more CPU.
    OdomF2M/MaxSize: "1000"
""")

run('ros2', 'run', 'rtabmap_odom', 'rgbd_odometry',
    '--ros-args',
    '--params-file', VODOM_YAML,
    '--remap', 'rgb/image:=/camera/camera/color/image_raw',
    '--remap', 'rgb/camera_info:=/camera/camera/color/camera_info',
    '--remap', 'depth/image:=/camera/camera/aligned_depth_to_color/image_raw',
    log='/tmp/slam_vodom.log')

# Poll — catch immediate crashes
print('  Polling for visual odometry...', end='', flush=True)
vodom_proc = PROCS[-1]
for i in range(12):
    time.sleep(1)
    print('.', end='', flush=True)
    if vodom_proc.poll() is not None:
        print(f'\n  ✗ Visual odometry crashed after {i+1}s!')
        try:
            lines = open('/tmp/slam_vodom.log').readlines()
            print('  Last log lines:')
            for l in lines[-15:]:
                print('   ', l.rstrip())
        except Exception:
            pass
        cleanup()
    # Consider it up once it's logged its params
    try:
        log_text = open('/tmp/slam_vodom.log').read()
        if 'stereoParams_' in log_text:
            print(f' ✓ (up at {i+1}s)')
            break
    except Exception:
        pass
else:
    print(' ✓ (process alive)')
print()

# ── [5/5] RTAB-MAP SLAM ───────────────────────────────────────────────────
print('[5/5] RTAB-Map SLAM...')

if args.localize:
    rtab_extra = ['--localization']
    incremental = 'false'
    print('  Mode: LOCALIZATION')
elif args.keep:
    rtab_extra = []
    incremental = 'true'
    print('  Mode: MAPPING (resume)')
else:
    rtab_extra = ['--delete_db_on_start']
    incremental = 'true'
    print('  Mode: MAPPING (fresh)')

# ROS2 CLI parses -p values with yaml.safe_load() internally.
# This means: 0.05 → float, true/false → bool, integers → int, text → str.
# subprocess.Popen passes args as raw bytes to the kernel, so ROS2 receives
# them exactly as written. The trick for string params that look like
# bool/number is YAML single-quoting: "'false'" → yaml parses as str "false".

run('ros2', 'run', 'rtabmap_slam', 'rtabmap',
    '--ros-args',
    # Native ROS2 typed params (yaml.safe_load does the right thing)
    '-p', 'frame_id:=base_link',
    '-p', 'odom_frame_id:=odom',
    '-p', 'map_frame_id:=map',
    '-p', 'subscribe_depth:=true',
    '-p', 'subscribe_rgb:=true',
    '-p', 'subscribe_odom_info:=false',
    '-p', 'approx_sync:=true',
    '-p', f'database_path:={DB_PATH}',
    '-p', 'wait_for_transform:=0.5',
    '-p', 'queue_size:=30',
    '-p', 'topic_queue_size:=30',
    # Grid/* — native double/bool in rtabmap_ros; yaml parses correctly
    "-p", "Grid/CellSize:='0.05'",
    "-p", "Grid/RangeMax:='5.0'",
    "-p", "Grid/MaxObstacleHeight:='0.6'",
    "-p", "Grid/MinObstacleHeight:='0.02'",
    "-p", "Grid/DepthDecimation:='2'",
    "-p", "Grid/FromDepth:='true'",
    # RTAB-Map internal string params — single-quoted so yaml returns str
    "-p", "Rtabmap/DetectionRate:='1.0'",
    "-p", "RGBD/LinearUpdate:='0.2'",
    "-p", "RGBD/AngularUpdate:='0.2'",
    "-p", "Vis/FeatureType:='6'",
    "-p", "Vis/MaxFeatures:='500'",
    "-p", "Vis/MinInliers:='6'",
    "-p", "Vis/MaxDepth:='4.0'",
    "-p", "Vis/MinDepth:='0.3'",
    "-p", "Kp/DetectorStrategy:='6'",
    "-p", "Kp/MaxFeatures:='500'",
    "-p", "Reg/Force3DoF:='true'",
    "-p", "RGBD/OptimizeFromGraphEnd:='false'",
    "-p", "RGBD/ProximityBySpace:='true'",
    "-p", "Mem/NotLinkedNodesKept:='false'",
    "-p", f"Mem/IncrementalMemory:='{incremental}'",
    '--remap', 'rgb/image:=/camera/camera/color/image_raw',
    '--remap', 'rgb/camera_info:=/camera/camera/color/camera_info',
    '--remap', 'depth/image:=/camera/camera/aligned_depth_to_color/image_raw',
    '--remap', 'grid_map:=/map',
    '--', *rtab_extra,
    log='/tmp/slam_rtabmap.log')

print('  Polling for RTAB-Map...', end='', flush=True)
rtab_proc = PROCS[-1]
for i in range(15):
    time.sleep(1)
    print('.', end='', flush=True)
    if rtab_proc.poll() is not None:
        print(f'\n  ✗ RTAB-Map crashed after {i+1}s!')
        try:
            log_text = open('/tmp/slam_rtabmap.log').read()
            lines = log_text.splitlines()
            print('  Last log lines:')
            for l in lines[-25:]:
                print('   ', l)
            # Parse the specific InvalidParameterTypeException for actionable advice
            import re
            m = re.search(r"parameter '([^']+)' has invalid type.*parameter \{[^}]+\} is of type \{(\w+)\}", log_text)
            if m:
                param, declared_type = m.group(1), m.group(2)
                print()
                print(f'  ► PARAM TYPE MISMATCH: {param!r} is declared as {declared_type!r}')
                if declared_type == 'string':
                    print(f'    Fix: value must be single-quoted in the -p arg')
                    print(f'    e.g.  "-p", "{param}:=\'value\'"')
                elif declared_type == 'double':
                    print(f'    Fix: value must be unquoted float')
                    print(f'    e.g.  "-p", "{param}:=0.05"')
                elif declared_type == 'bool':
                    print(f'    Fix: value must be unquoted true/false')
                    print(f'    e.g.  "-p", "{param}:=true"')
        except Exception as e:
            print(f'  (could not read log: {e})')
        cleanup()
    try:
        log_text = open('/tmp/slam_rtabmap.log').read()
        if 'rtabmap:' in log_text:
            print(f' ✓ (up at {i+1}s)')
            break
    except Exception:
        pass
else:
    print(' ✓ (process alive)')

# ── Ready ─────────────────────────────────────────────────────────────────
print()
print('=========================================')
print('  ✓✓✓ SLAM READY ✓✓✓')
print('=========================================')
print()
print('KEY TOPICS:')
print('  /map                 ← occupancy grid')
print('  /rtabmap/cloud_map   ← 3D point cloud map')
print('  /odom                ← odometry')
print('  /scan                ← laser scan')
print()
print('MOVE THE CAMERA SLOWLY to build the map.')
print('Point at a textured surface — not a blank wall.')
print()
print('VISUALIZER: open a second terminal and run:')
print('  bash slam_laptop.sh')
print()
print('LOGS:')
print('  tail -f /tmp/slam_vodom.log')
print('  tail -f /tmp/slam_rtabmap.log')
print()
print('Press Ctrl+C to stop all nodes')
print('=========================================')

# ── Monitor loop ──────────────────────────────────────────────────────────
WATCHED = [
    ('RTAB-Map',     rtab_proc,  '/tmp/slam_rtabmap.log'),
    ('VisualOdom',   vodom_proc, '/tmp/slam_vodom.log'),
]

tick = 0
while True:
    time.sleep(5)
    tick += 1
    for name, proc, log in WATCHED:
        if proc.poll() is not None:
            print(f'\n⚠  {name} died!')
            try:
                lines = open(log).readlines()
                print(f'  Last lines of {log}:')
                for l in lines[-15:]:
                    print('   ', l.rstrip())
            except Exception:
                pass
            cleanup()
    if tick % 12 == 0:
        n = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
        count = len(n.stdout.strip().splitlines())
        print(f'[{time.strftime("%H:%M:%S")}] Running — {count} nodes active')