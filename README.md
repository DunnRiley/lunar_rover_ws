# Lunar Rover — Lunabotics / NASA Competition Rover

This is the software stack for our autonomous lunar excavation rover. It runs across two machines: a **mini PC** on the rover (hostname `cheese`, IP `192.168.0.102`) and a **laptop** used as mission control. Both must be on the same WiFi network.

---

## Arduino Serial Protocol

All commands follow this 7-byte format:

```
[0xAA] [Device] [Speed] [Direction] [LoByte] [Checksum] [0x55]
Checksum = Device XOR Speed XOR Direction XOR LoByte
```

| Command | Device Byte | Description |
|---|---|---|
| Left motors (FL+BL) | `0x05` | Speed 0–255, Direction 0/1 |
| Right motors (FR+BR) | `0x06` | Speed 0–255, Direction 0/1 |
| Both actuators | `0x08` | Speed 0–255, Direction 0/1 |
| Servo | `0x11` | Speed byte = angle. 90=stop, 45=CCW, 135=CW |
| Actuator → DIG 1 | `0xA7` | Moves to dig position 1 |
| Actuator → DIG 2 | `0x93` | Moves to dig position 2 |
| Actuator → DRIVE | `0xA9` | Raises to drive/transport position |
| Actuator → DUMP | `0xB3` | Moves to dump position |
| Calibrate actuator | `0xCA` | Drives to hard stop, zeros encoder |
| Distance drive (dual) | `0xDC` | Encoder-based straight drive |
| Load left encoder | `0xC8` | Pre-load left wheel target |
| Load right encoder | `0xC9` | Pre-load right wheel target |
| Start isolated turn | `0xE8` | Each side stops at its own target |
| STOP ALL | `0xFF` | Emergency stop everything |

**Telemetry back from Arduino (Serial2, 115200 baud):**
```
[0xAA] [ax 4B] [ay 4B] [az 4B] [gx 4B] [gy 4B] [gz 4B] [0xA5] [enc 2B] [checksum] [0x55]
All IMU values are int32 = physical_value × 1000
Encoder is uint16, centred at 32000
```

---

## How to Run

### 1. Connect to miniPC

```bash
ssh cheese@192.168.0.102
```

### 2. Build the workspace (first time or after code changes)

```bash
cd ~/lunar_rover_ws
colcon build
source install/setup.bash
```

### 3. Flash Arduino

```bash
arduino-cli compile --upload -p /dev/ttyACM0 --fqbn arduino:avr:mega .
```

### 4. Launch everything on miniPC

```bash
bash ~/lunar_rover_ws/full_launch_minipc.sh
```

This starts: TF tree, D435 camera, image pipelines, rear stereo camera, joy_node, joy_to_arduino, and the autonomous nav nodes.

To disable the nav nodes (lighter launch):
```bash
NAV=0 bash ~/lunar_rover_ws/full_launch_minipc.sh
```

To add a competition delay buffer (e.g. 5 seconds) to the camera streams:
```bash
DELAY_SEC=5.0 bash ~/lunar_rover_ws/full_launch_minipc.sh
```

### 5. Launch mission control on laptop

```bash
bash ~/lunar_rover_ws/full_launch_laptop.sh
```

Or to also SSH-start the miniPC automatically:
```bash
bash ~/lunar_rover_ws/full_launch_laptop.sh --start-minipc
```

### 6. Run a mission

```bash
# Dry-run to validate YAML first
bash ~/lunar_rover_ws/run_mission.sh --dry-run mission.yaml

# Run it
bash ~/lunar_rover_ws/run_mission.sh mission.yaml

# Or use the GUI: MISSION tab → Load → START MISSION
```

Pre-built mission files:
- `mission.yaml` — Full excavation sequence (edit distances before use)
- `dig.yaml` — Digging run only
- `dump.yaml` — Dump run only

**⚠️ Always check `distance_m` values in YAML are in metres, not millimetres.**

---

## Teleop (Joystick)

Plug Xbox controller into the **miniPC USB port**, then:

```bash
# Via GUI: CONTROL tab → START TELEOP
# Or manually on miniPC:
ros2 run joy joy_node &
python3 ~/lunar_rover_ws/joy_to_arduino.py
```

| Control | Action |
|---|---|
| Left stick Y | Left wheels (tank drive) |
| Right stick Y | Right wheels (tank drive) |
| LT hold | Actuator extend |
| RT hold | Actuator retract |
| LB hold | Servo CCW |
| RB hold | Servo CW |
| A | Actuator → DIG 2 |
| B | Actuator → DIG 1 |
| Y | Actuator → DRIVE position |
| X | Actuator calibrate / dump |
| Start | Emergency stop toggle |

---

## ROS Environment Variables

These must be set on **both machines**. Add to `~/.bashrc` if not already there:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Run once on miniPC to add them automatically:
```bash
bash ~/lunar_rover_ws/fix_minipc_env.sh
```

---

## Key Topics

| Topic | Direction | Description |
|---|---|---|
| `/camera/color/stream/compressed` | miniPC → laptop | Front RGB @ 6fps |
| `/camera/depth/stream/compressed` | miniPC → laptop | Front depth @ 3fps |
| `/camera_rear/stream/compressed` | miniPC → laptop | Rear stereo @ 10fps |
| `/imu/gyro_deg_s` | Arduino → miniPC | Gyro [gx, gy, gz] deg/s |
| `/imu/accel_ms2` | Arduino → miniPC | Accel [ax, ay, az] m/s² |
| `/nav/encoder_raw` | Arduino → miniPC | Actuator encoder count |
| `/mission/start` | laptop → miniPC | Bool: start/abort mission |
| `/mission/status` | miniPC → laptop | JSON step progress |
| `/nav/arduino_dist_cmd` | miniPC → Arduino | Float32 metres to drive |
| `/nav/arduino_turn_cmd` | miniPC → Arduino | Float32Array: arc_mm, speed, CW |
| `/cmd_vel` | any → Arduino bridge | Twist for arc/turn motion |

---

## Cameras

**Front — Intel RealSense D435**
- USB 3.0 required (blue port)
- Topics: `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`
- If it fails: `bash DiagnosticAndTesting/diagnose_camera.sh`
- USB stability fix: `sudo bash DiagnosticAndTesting/fix_camera_usb.sh`

**Rear — IFWATER 3D Stereo USB (1600×600 side-by-side)**
- Appears as a single `/dev/videoX` device
- Udev symlink for stable device path: `bash ~/lunar_rover_ws/fix_stereo_udev.sh`
- After running, camera is always at `/dev/video_stereo`
- To restart just the rear camera: `bash ~/lunar_rover_ws/restart_rear_camera.sh`

---

## Diagnostics

```bash
# Check ROS network and streaming
bash ~/lunar_rover_ws/check_streaming.sh

# Check if all SLAM/nav nodes are up
bash DiagnosticAndTesting/slam_check_nodes.sh

# Check TF tree
bash DiagnosticAndTesting/check_tf.sh

# View all logs
bash DiagnosticAndTesting/view_logs.sh

# Find stereo camera device
bash DiagnosticAndTesting/find_camera_devices.sh
```
---

## Other Useful Commands

```bash
# Record depth camera bag
rs-record -f depth_only.bag -t 5

# Play a sound (competition signal)
speaker-test -t sine -f 440

# Kill all nav/camera nodes
pkill -f joy_to_arduino
pkill -f arduino_teleop_controller
pkill -f optimized_image_pipeline

# Build only a specific package
colcon build --packages-select lunar_robot_hardware
```
