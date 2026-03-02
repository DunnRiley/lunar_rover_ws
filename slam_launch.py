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

FIX NOTE (2024):
  The original script used `ros2 launch realsense2_camera rs_launch.py`
  which fails if the `launch` ros2 verb is not available in the subprocess
  environment (common issue — ros2launch extension not in PATH for child procs).
  
  Camera is now launched via `ros2 run realsense2_camera realsense2_camera_node`
  with all parameters passed as --ros-args -p flags. This is more reliable and
  doesn't depend on the launch infrastructure being available.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

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

def run_shell(cmd_str, log=None, check_delay=None):
    """
    Launch via bash -c so the full environment (including ros2 launch verb)
    is available. Use this for commands that need ros2 launch.
    """
    env = os.environ.copy()
    stdout = open(log, 'w') if log else None
    stderr = open(log, 'a') if log else None
    p = subprocess.Popen(
        ['bash', '-c', cmd_str],
        env=env, stdout=stdout, stderr=stderr
    )
    PROCS.append(p)
    if check_delay:
        time.sleep(check_delay)
        if p.poll() is not None:
            print(f'\n✗ Process died (shell cmd): {cmd_str[:60]}')
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
    """Poll TF tree."""
    print(f'  Waiting for TF {source}→{target}...', end='', flush=True)
    for _ in range(timeout):
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
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5
            )
            if 'robot_state_publisher' in result.stdout:
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
ROS_DISTRO = 'unknown'
for ros_setup in ['/opt/ros/jazzy/setup.bash', '/opt/ros/humble/setup.bash']:
    if os.path.exists(ros_setup):
        ROS_DISTRO = 'jazzy' if 'jazzy' in ros_setup else 'humble'
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

for child in ['camera_depth_optical_frame', 'camera_color_optical_frame']:
    run('ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        '--x', '0', '--y', '0', '--z', '0',
        '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
        '--frame-id', 'camera_link',
        '--child-frame-id', child,
        log='/dev/null')

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
print('  Launching via ros2 run (avoids launch-verb availability issues)...')

# ── APPROACH: ros2 run instead of ros2 launch ──────────────────────────
#
# WHY: `ros2 launch` requires the ros2launch Python entry point to be
# registered in the subprocess environment. When Python spawns a child
# process, even with env=os.environ.copy(), the ros2 launch verb is
# sometimes missing if the ROS2 overlay wasn't sourced in the same shell.
#
# ros2 run works reliably because it only needs the ament index, which
# IS in the environment after we sourced setup.bash above.
#
# The realsense2_camera node accepts all the same parameters that
# rs_launch.py sets, just passed directly as --ros-args -p flags.
# We push the node into the "camera" namespace to match the topic
# structure (/camera/camera/color/image_raw etc.) that the rest of
# the pipeline expects.

run('ros2', 'run', 'realsense2_camera', 'realsense2_camera_node',
    '--ros-args',
    '--remap', '__ns:=/camera',
    '--remap', '__node:=camera',
    '-p', 'camera_name:=camera',
    '-p', 'enable_depth:=true',
    '-p', 'enable_color:=true',
    '-p', 'enable_infra1:=false',
    '-p', 'enable_infra2:=false',
    '-p', 'enable_sync:=true',
    '-p', 'align_depth.enable:=true',
    '-p', 'pointcloud.enable:=true',
    '-p', 'depth_module.profile:=640x480x15',
    '-p', 'rgb_camera.profile:=640x480x15',
    log='/tmp/slam_camera.log',
    check_delay=3)

# Wait for the color image topic — primary liveness check
color_topic = '/camera/camera/color/image_raw'
if not wait_for_topic(color_topic, timeout=30):
    print()
    print('  ✗ Camera failed to publish. Checking common causes...')
    print()
    # Try to give a helpful diagnosis from the log
    try:
        log_text = open('/tmp/slam_camera.log').read()
        if 'permission denied' in log_text.lower() or 'LIBUSB' in log_text:
            print('  ► USB/permission error detected.')
            print('    Fix: sudo chmod a+rw /dev/bus/usb/*/*')
            print('    Or:  sudo usermod -aG plugdev $USER  (then log out/in)')
        elif 'No device found' in log_text or 'no RealSense' in log_text.lower():
            print('  ► Camera not detected on USB.')
            print('    Check: lsusb | grep Intel')
            print('    Try a different USB3 port (blue port required for D435).')
        elif 'could not load library' in log_text.lower() or 'librealsense' in log_text.lower():
            print('  ► librealsense not installed or wrong version.')
            print('    Fix: sudo apt install ros-jazzy-realsense2-camera')
            print('         sudo apt install librealsense2-dkms librealsense2-utils')
        else:
            print('  ► Unknown error. Last 25 lines of /tmp/slam_camera.log:')
            lines = log_text.splitlines()
            for l in lines[-25:]:
                print('    ', l)
    except Exception as e:
        print(f'  (could not read log: {e})')
    print()
    print('  Full log: tail -50 /tmp/slam_camera.log')
    cleanup()

# Also wait for depth (aligned) — needed by RTAB-Map
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
print(f'  Waiting for aligned depth...', end='', flush=True)
for _ in range(15):
    result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
    if depth_topic in result.stdout:
        print(' ✓')
        break
    print('.', end='', flush=True)
    time.sleep(1)
else:
    print(' ✗ (aligned depth timeout — align_depth may not have enabled)')
    print('  WARNING: Continuing but RTAB-Map may fail.')

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

VODOM_YAML = '/tmp/vodom_params.yaml'
with open(VODOM_YAML, 'w') as yf:
    yf.write("""rgbd_odometry:
  ros__parameters:
    frame_id: "base_link"
    odom_frame_id: "odom"
    publish_tf: true
    approx_sync: true
    approx_sync_max_interval: 0.2
    wait_for_transform: 0.5
    queue_size: 30
    Odom/Strategy: "0"
    Odom/GuessMotion: "true"
    Odom/ResetCountdown: "0"
    Vis/FeatureType: "6"
    Vis/MaxFeatures: "500"
    Vis/MinInliers: "6"
    Vis/MaxDepth: "4.0"
    Vis/MinDepth: "0.3"
    Odom/FilteringStrategy: "1"
    OdomF2M/MaxSize: "1000"
""")

run('ros2', 'run', 'rtabmap_odom', 'rgbd_odometry',
    '--ros-args',
    '--params-file', VODOM_YAML,
    '--remap', 'rgb/image:=/camera/camera/color/image_raw',
    '--remap', 'rgb/camera_info:=/camera/camera/color/camera_info',
    '--remap', 'depth/image:=/camera/camera/aligned_depth_to_color/image_raw',
    log='/tmp/slam_vodom.log')

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
    try:
        log_text = open('/tmp/slam_vodom.log').read()
        if 'stereoParams_' in log_text or 'Odom initialized' in log_text:
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

run('ros2', 'run', 'rtabmap_slam', 'rtabmap',
    '--ros-args',
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
    "-p", "Grid/CellSize:='0.05'",
    "-p", "Grid/RangeMax:='5.0'",
    "-p", "Grid/MaxObstacleHeight:='0.6'",
    "-p", "Grid/MinObstacleHeight:='0.02'",
    "-p", "Grid/DepthDecimation:='2'",
    "-p", "Grid/FromDepth:='true'",
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
            import re
            m = re.search(r"parameter '([^']+)' has invalid type.*parameter \{[^}]+\} is of type \{(\w+)\}", log_text)
            if m:
                param, declared_type = m.group(1), m.group(2)
                print()
                print(f'  ► PARAM TYPE MISMATCH: {param!r} is declared as {declared_type!r}')
                if declared_type == 'string':
                    print(f'    Fix: value must be single-quoted: "-p", "{param}:=\'value\'"')
                elif declared_type == 'double':
                    print(f'    Fix: value must be unquoted float: "-p", "{param}:=0.05"')
                elif declared_type == 'bool':
                    print(f'    Fix: value must be unquoted: "-p", "{param}:=true"')
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