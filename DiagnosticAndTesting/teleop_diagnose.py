#!/usr/bin/env python3
"""
teleop_diagnose.py  —  run this on the miniPC to find the teleop problem.
Checks everything needed for motor control and prints a clear pass/fail report.

Usage:
  python3 teleop_diagnose.py
"""

import os
import sys
import glob
import subprocess
import time

PASS  = '\033[92m  ✓\033[0m'
FAIL  = '\033[91m  ✗\033[0m'
WARN  = '\033[93m  ⚠\033[0m'
HEAD  = '\033[1m'
RESET = '\033[0m'

issues = []

def check(label, ok, fix=''):
    sym = PASS if ok else FAIL
    print(f'{sym} {label}')
    if not ok and fix:
        print(f'      → {fix}')
        issues.append(f'{label}: {fix}')
    return ok

def section(title):
    print(f'\n{HEAD}── {title} ──{RESET}')


# ══════════════════════════════════════════════════════════════════════════
section('ARDUINO / SERIAL')
# ══════════════════════════════════════════════════════════════════════════

acm_ports = glob.glob('/dev/ttyACM*')
usb_ports = glob.glob('/dev/ttyUSB*')
all_ports  = acm_ports + usb_ports

check('Arduino port exists', bool(all_ports),
      'Connect Arduino USB cable.  Run: ls /dev/ttyACM* /dev/ttyUSB*')

if all_ports:
    port = all_ports[0]
    print(f'      Found: {all_ports}  (using {port})')

    # Check permissions
    readable = os.access(port, os.R_OK | os.W_OK)
    check(f'Serial port {port} is readable/writable', readable,
          f'Run: sudo usermod -aG dialout $USER  then log out and back in\n'
          f'      Or temporary fix: sudo chmod 666 {port}')

# Check pyserial
try:
    import serial
    check('pyserial installed', True)
    ver = serial.__version__
    print(f'      version: {ver}')
except ImportError:
    check('pyserial installed', False,
          'pip3 install pyserial --break-system-packages')

# Quick serial open test
if all_ports and readable:
    try:
        import serial as _serial
        s = _serial.Serial(all_ports[0], 115200, timeout=0.5)
        s.close()
        check('Serial port opens without error', True)
    except Exception as e:
        check('Serial port opens without error', False, str(e))

# ══════════════════════════════════════════════════════════════════════════
section('ROS2 ENVIRONMENT')
# ══════════════════════════════════════════════════════════════════════════

ros_domain = os.environ.get('ROS_DOMAIN_ID', 'NOT SET')
check('ROS_DOMAIN_ID set', ros_domain == '42',
      f'Current: {ros_domain}  Expected: 42\n'
      f'      Add to ~/.bashrc:  export ROS_DOMAIN_ID=42')

ros_localhost = os.environ.get('ROS_LOCALHOST_ONLY', 'NOT SET')
check('ROS_LOCALHOST_ONLY=0', ros_localhost == '0',
      f'Current: {ros_localhost}\n'
      f'      Add to ~/.bashrc:  export ROS_LOCALHOST_ONLY=0')

disc_range = os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', 'NOT SET')
check('ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET', disc_range == 'SUBNET',
      f'Current: {disc_range}\n'
      f'      Add to ~/.bashrc:  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET')

# Check ROS2 installed
r = subprocess.run(['which', 'ros2'], capture_output=True, text=True)
check('ros2 command found', r.returncode == 0,
      'Source ROS2:  source /opt/ros/jazzy/setup.bash')

# Check workspace
ws_setup = os.path.expanduser('~/lunar_rover_ws/install/setup.bash')
check('Workspace built (install/setup.bash exists)', os.path.exists(ws_setup),
      'cd ~/lunar_rover_ws && colcon build')

# ══════════════════════════════════════════════════════════════════════════
section('ROS2 TOPICS  (requires sourced ROS + workspace)')
# ══════════════════════════════════════════════════════════════════════════

env_cmd = 'source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null'
if os.path.exists(ws_setup):
    env_cmd += f' && source {ws_setup}'
env_cmd += ' && '

def ros_run(cmd, timeout=5):
    full = f'bash -c "{env_cmd} {cmd}"'
    r = subprocess.run(full, shell=True, capture_output=True, text=True, timeout=timeout)
    return r.stdout.strip(), r.returncode

try:
    topics, rc = ros_run('ros2 topic list 2>/dev/null')
    has_joy = '/joy' in topics.split('\n')
    check('/joy topic exists (laptop joy_node running)', has_joy,
          'On laptop: ros2 run joy joy_node\n'
          '      Or run full_launch_laptop.sh first')

    if has_joy:
        # Check if it's actually publishing
        result, _ = ros_run('timeout 2 ros2 topic hz /joy 2>&1 | head -3', timeout=5)
        publishing = 'average rate' in result
        check('/joy is actively publishing data', publishing,
              'joy topic exists but no data — plug in controller and press a button')

    has_cmd = '/cmd_vel' in topics.split('\n')
    print(f'{"  ✓" if has_cmd else "  ·"}  /cmd_vel topic {"exists" if has_cmd else "not yet (ok if joy_to_arduino not running)"}')

except subprocess.TimeoutExpired:
    print(f'{WARN} ROS topic check timed out (ROS may not be running)')
except Exception as e:
    print(f'{WARN} Could not check topics: {e}')

# ══════════════════════════════════════════════════════════════════════════
section('SCRIPTS')
# ══════════════════════════════════════════════════════════════════════════

ws = os.path.expanduser('~/lunar_rover_ws')
scripts = [
    'joy_to_arduino.py',
    'minipc_teleop_standalone.py',
    'full_launch_minipc.sh',
    'optimized_image_pipeline.py',
]
for s in scripts:
    path = os.path.join(ws, s)
    check(f'{s} exists in workspace', os.path.exists(path),
          f'Copy file to {ws}/')

# Check joy_to_arduino for known duplication bug
j2a = os.path.join(ws, 'joy_to_arduino.py')
if os.path.exists(j2a):
    with open(j2a) as f:
        content = f.read()
    # The bug: docstring appears twice
    dup = content.count('import rclpy\nfrom rclpy.node') > 1 or \
          content.count('#!/usr/bin/env python3') > 1
    check('joy_to_arduino.py has no duplication bug', not dup,
          'File has duplicate content — replace with the fixed version!')

# ══════════════════════════════════════════════════════════════════════════
section('CONTROLLER (if plugged into miniPC)')
# ══════════════════════════════════════════════════════════════════════════

js_devs = glob.glob('/dev/input/js*')
if js_devs:
    check(f'Joystick device found ({js_devs[0]})', True)
    # Check evdev for controller name
    try:
        r = subprocess.run(['cat', '/proc/bus/input/devices'],
                           capture_output=True, text=True)
        if 'Xbox' in r.stdout or 'Microsoft' in r.stdout or 'X-Box' in r.stdout:
            check('Xbox controller detected', True)
        else:
            print(f'{WARN} Controller found but name not confirmed as Xbox')
            print('      Run: cat /proc/bus/input/devices | grep -A5 Joystick')
    except Exception:
        pass
else:
    print(f'  ·  No joystick at /dev/input/js* — controller not plugged into miniPC')
    print('     (This is fine if controller is on laptop)')

# ══════════════════════════════════════════════════════════════════════════
section('NETWORK')
# ══════════════════════════════════════════════════════════════════════════

# Check if laptop is reachable
r = subprocess.run(['ping', '-c1', '-W2', '192.168.0.102'],
                   capture_output=True)
is_minipc = r.returncode == 0
# Try pinging common laptop IP
r2 = subprocess.run(['ip', 'route'], capture_output=True, text=True)
print(f'  Network routes: {r2.stdout.strip()[:120]}')

r3 = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
print(f'  This machine IP: {r3.stdout.strip()}')

# ══════════════════════════════════════════════════════════════════════════
section('SUMMARY')
# ══════════════════════════════════════════════════════════════════════════

if issues:
    print(f'\n{HEAD}Found {len(issues)} issue(s) to fix:{RESET}')
    for i, issue in enumerate(issues, 1):
        print(f'  {i}. {issue}')
    print()
else:
    print(f'\n{PASS} Everything looks good!')
    print('  If teleop still does not work, run:')
    print('    python3 ~/lunar_rover_ws/minipc_teleop_standalone.py')
    print('  and watch for diagnostic output.\n')

print('─' * 55)
print('Quick manual test commands:')
print('  # Check if /joy arrives from laptop:')
print('  ros2 topic echo /joy --once')
print()
print('  # Test serial directly (replace ACM0 if needed):')
print("  python3 -c \"import serial,time; s=serial.Serial('/dev/ttyACM0',115200); time.sleep(2); s.write(bytes([0xAA,0x05,100,0,0x55])); print('sent LEFT forward')\"")
print()
print('  # Watch joy_to_arduino logs:')
print('  tail -f /tmp/rover_arduino.log')
print('─' * 55)