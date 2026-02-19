#!/bin/bash
# ============================================================================
#  MINI PC: RTAB-Map SLAM + Nav2 (FIXED)
#
#  Fixes:
#  - Dead-reckoning odometry added (no encoder dependency)
#  - Nav2 launched and wired to RTAB-Map map frame
#  - Topic remappings corrected throughout
#  - Proper startup sequencing with health checks
#  - Depth→LaserScan conversion for Nav2 costmaps
#
#  MODES:
#    bash slam_minipc.sh map          → Fresh map (deletes old DB)
#    bash slam_minipc.sh map keep     → Resume / extend existing map
#    bash slam_minipc.sh localize     → Load saved map, localize only
#
#  MAP FILE: ~/.ros/rtabmap_rover.db
# ============================================================================

MODE=${1:-map}
KEEP_MAP=${2:-""}
DB_PATH="$HOME/.ros/rtabmap_rover.db"

# ── ROS setup ─────────────────────────────────────────────────────────────
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash && ROS_DISTRO=jazzy
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash && ROS_DISTRO=humble
else
    echo "✗ No ROS2 found" && exit 1
fi

[ -f ~/lunar_rover_ws/install/setup.bash ] && source ~/lunar_rover_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0

# ── Print header ──────────────────────────────────────────────────────────
echo "========================================="
case "$MODE" in
    localize) echo "  MINI PC: SLAM — LOCALIZATION MODE" ;;
    *)
        if [ "$KEEP_MAP" = "keep" ]; then
            echo "  MINI PC: SLAM — MAPPING (RESUME)"
        else
            echo "  MINI PC: SLAM — MAPPING (FRESH)"
        fi ;;
esac
echo "========================================="
echo "ROS2: $ROS_DISTRO  |  DOMAIN: $ROS_DOMAIN_ID"
echo ""

# ── Dependency checks ─────────────────────────────────────────────────────
echo "Checking dependencies..."
MISSING=0

check_pkg() {
    if ! ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo "  ✗ Missing: $1"
        MISSING=$((MISSING+1))
    else
        echo "  ✓ $1"
    fi
}

check_pkg rtabmap_ros
check_pkg rtabmap_slam
check_pkg rtabmap_odom
check_pkg nav2_bringup
check_pkg depthimage_to_laserscan
check_pkg realsense2_camera

if [ $MISSING -gt 0 ]; then
    echo ""
    echo "Install missing packages:"
    echo "  sudo apt install ros-${ROS_DISTRO}-rtabmap-ros \\"
    echo "                   ros-${ROS_DISTRO}-nav2-bringup \\"
    echo "                   ros-${ROS_DISTRO}-depthimage-to-laserscan"
    exit 1
fi
echo ""

# ── Map file handling ─────────────────────────────────────────────────────
if [ "$MODE" = "localize" ] && [ ! -f "$DB_PATH" ]; then
    echo "✗ No map at $DB_PATH — run mapping first"
    exit 1
fi
[ -f "$DB_PATH" ] && echo "Map file: $DB_PATH ($(du -h "$DB_PATH" | cut -f1))" || echo "No existing map (will create)"
echo ""

# ── Kill old processes ────────────────────────────────────────────────────
echo "Stopping old nodes..."
pkill -f realsense2_camera_node 2>/dev/null
pkill -f rgbd_odometry         2>/dev/null
pkill -f rtabmap               2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f static_transform_pub  2>/dev/null
pkill -f depthimage_to_laserscan 2>/dev/null
pkill -f nav2_bringup          2>/dev/null
pkill -f lifecycle_manager     2>/dev/null
pkill -f simple_odom_publisher 2>/dev/null
sleep 2
echo "✓ Clean"
echo ""

trap 'echo ""; echo "Shutting down..."; kill 0; wait; echo "✓ Stopped"; exit' SIGINT SIGTERM

# ── Write Nav2 params ─────────────────────────────────────────────────────
NAV2_PARAMS=/tmp/nav2_params_slam.yaml
cat > "$NAV2_PARAMS" << 'NAV2_EOF'
# Nav2 parameters tuned for RTAB-Map integration
# Fixed frame = map (published by RTAB-Map)

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    navigators: ['navigate_to_pose', 'navigate_through_poses']
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::ProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 15.0
    general_goal_checker:
      plugin: "nav2_controller::GoalChecker"
      xy_goal_tolerance: 0.30
      yaw_goal_tolerance: 0.30
      stateful: true
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.25
      lookahead_dist: 0.7
      min_lookahead_dist: 0.3
      max_lookahead_dist: 1.2
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 0.4
      transform_tolerance: 0.5
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.8
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.5
      use_regulated_linear_velocity_scaling: true
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 0.9
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.2
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 0.8
      max_robot_pose_search_dist: 10.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.05
      robot_radius: 0.30
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: false
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 1.5
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /camera/camera/depth/color/points
          max_obstacle_height: 1.5
          clearing: true
          marking: true
          data_type: "PointCloud2"
          raytrace_max_range: 4.0
          raytrace_min_range: 0.1
          obstacle_max_range: 3.5
          obstacle_min_range: 0.1
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.45
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.30
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        # Reads /map topic published by RTAB-Map
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: pointcloud
        pointcloud:
          topic: /camera/camera/depth/color/points
          max_obstacle_height: 1.5
          clearing: true
          marking: true
          data_type: "PointCloud2"
          raytrace_max_range: 4.0
          obstacle_max_range: 3.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.45
      always_send_full_costmap: true

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: false
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.5
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.4
    min_rotational_vel: 0.1
    rotational_acc_lim: 0.8

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 0

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.4, 0.0, 0.8]
    min_velocity: [-0.4, 0.0, -0.8]
    max_accel: [0.8, 0.0, 1.5]
    max_decel: [-0.8, 0.0, -1.5]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: ""   # empty = no static map file; RTAB-Map publishes /map

amcl:
  ros__parameters:
    use_sim_time: false
NAV2_EOF

echo "✓ Nav2 params written to $NAV2_PARAMS"

# ── URDF ─────────────────────────────────────────────────────────────────
URDF='<?xml version="1.0"?>
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
</robot>'

# ── [1/6] TF TREE ─────────────────────────────────────────────────────────
echo "[1/6] TF tree + robot description..."

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$URDF" -p publish_frequency:=50.0 \
    2>/tmp/slam_rsp.log &

sleep 1

# Camera optical frame transforms
# ROS2 Jazzy static_transform_publisher uses CLI flags (not --ros-args -p)
ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 \
    --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
    --frame-id camera_link \
    --child-frame-id camera_depth_optical_frame \
    2>/dev/null &

ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 \
    --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 \
    --frame-id camera_link \
    --child-frame-id camera_color_optical_frame \
    2>/dev/null &

sleep 1
echo "  ✓ TF tree ready"
echo ""

# ── [2/6] DEAD-RECKONING ODOMETRY ─────────────────────────────────────────
# Publishes /odom and base_footprint→base_link TF from /cmd_vel integration.
# Replace with encoder odometry when hardware is available.
echo "[2/6] Dead-reckoning odometry (cmd_vel integration)..."

python3 - << 'ODOM_EOF' &
#!/usr/bin/env python3
"""
Simple dead-reckoning odometry from cmd_vel.
Publishes /odom and the odom→base_footprint TF.
Replace this with encoder-based odometry for better accuracy.
"""
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class DeadReckonOdom(Node):
    def __init__(self):
        super().__init__('dead_reckoning_odom')
        self.x = self.y = self.th = self.vx = self.vth = 0.0
        self.last = self.get_clock().now()
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom', 50)
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.create_timer(0.02, self.update)   # 50 Hz
        self.get_logger().info('Dead-reckoning odometry running (50 Hz)')

    def cb(self, msg):
        self.vx  = msg.linear.x
        self.vth = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds / 1e9
        self.last = now
        if dt <= 0 or dt > 0.5:
            return

        dx  = self.vx * math.cos(self.th) * dt
        dy  = self.vx * math.sin(self.th) * dt
        dth = self.vth * dt
        self.x  += dx
        self.y  += dy
        self.th += dth
        while self.th >  math.pi: self.th -= 2*math.pi
        while self.th < -math.pi: self.th += 2*math.pi

        qz = math.sin(self.th/2.0)
        qw = math.cos(self.th/2.0)
        stamp = now.to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.br.sendTransform(t)

        o = Odometry()
        o.header.stamp    = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id  = 'base_footprint'
        o.pose.pose.position.x    = self.x
        o.pose.pose.position.y    = self.y
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x  = self.vx
        o.twist.twist.angular.z = self.vth
        # Covariance — diagonal, modest confidence
        o.pose.covariance[0]  = 0.05
        o.pose.covariance[7]  = 0.05
        o.pose.covariance[35] = 0.1
        o.twist.covariance[0] = 0.05
        o.twist.covariance[35]= 0.1
        self.pub.publish(o)

rclpy.init()
node = DeadReckonOdom()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
ODOM_EOF

ODOM_PID=$!
sleep 2
echo "  ✓ Odometry publishing on /odom"
echo ""

# ── [3/6] CAMERA ─────────────────────────────────────────────────────────
echo "[3/6] D435 Camera (640×480 @ 15 fps)..."

ros2 launch realsense2_camera rs_launch.py \
    camera_name:=camera \
    camera_namespace:=camera \
    enable_depth:=true \
    enable_color:=true \
    enable_infra1:=false \
    enable_infra2:=false \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    enable_sync:=true \
    depth_module.profile:=640x480x15 \
    rgb_camera.profile:=640x480x15 \
    2>/tmp/slam_camera.log &

CAM_PID=$!
echo "  Waiting for camera (10 s)..."
sleep 10

if ! ps -p $CAM_PID >/dev/null 2>&1; then
    echo "  ✗ Camera failed! Check: tail /tmp/slam_camera.log"
    exit 1
fi

# Wait for image topic
for i in $(seq 1 20); do
    ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw$" && break
    sleep 1
done
ros2 topic list 2>/dev/null | grep -q "/camera/camera/color/image_raw$" \
    && echo "  ✓ Camera publishing (PID $CAM_PID)" \
    || { echo "  ✗ No camera topics! Check: tail /tmp/slam_camera.log"; exit 1; }

# ── Depth→LaserScan (for Nav2 costmap fallback) ───────────────────────────
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
    --ros-args \
    --remap depth/image_raw:=/camera/camera/aligned_depth_to_color/image_raw \
    --remap depth/camera_info:=/camera/camera/depth/camera_info \
    --remap scan:=/scan \
    -p scan_height:=1 \
    -p scan_time:=0.066 \
    -p range_min:=0.2 \
    -p range_max:=5.0 \
    -p output_frame_id:=camera_link \
    2>/tmp/slam_scan.log &
echo "  ✓ Depth→LaserScan on /scan"
echo ""

# ── [4/6] RTAB-MAP VISUAL ODOMETRY ───────────────────────────────────────
echo "[4/6] RTAB-Map RGB-D visual odometry..."

ros2 run rtabmap_odom rgbd_odometry \
    --ros-args \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=false \
    -p approx_sync:=true \
    -p approx_sync_max_interval:=0.15 \
    -p wait_for_transform:=0.3 \
    -p queue_size:=30 \
    -p Odom/Strategy:=0 \
    -p Vis/FeatureType:=6 \
    -p Vis/MaxFeatures:=600 \
    -p Vis/MinInliers:=8 \
    -p Vis/MaxDepth:="5.0" \
    -p Odom/ResetCountdown:=1 \
    -p Odom/GuessMotion:=false \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    2>/tmp/slam_vodom.log &

VODOM_PID=$!
echo "  Waiting for visual odometry (6 s)..."
sleep 6

if ! ps -p $VODOM_PID >/dev/null 2>&1; then
    echo "  ✗ Visual odometry failed!"
    tail -20 /tmp/slam_vodom.log
    exit 1
fi
echo "  ✓ Visual odometry running (PID $VODOM_PID)"
echo "  NOTE: Visual odom supplements but does NOT override /odom"
echo ""

# ── [5/6] RTAB-MAP SLAM ───────────────────────────────────────────────────
echo "[5/6] RTAB-Map SLAM..."

case "$MODE" in
    localize)
        RTAB_ARGS="--localization"
        MEM_PARAM="-p Mem/IncrementalMemory:=false"
        echo "  Mode: LOCALIZATION"
        ;;
    *)
        if [ "$KEEP_MAP" = "keep" ]; then
            RTAB_ARGS=""
            MEM_PARAM="-p Mem/IncrementalMemory:=true"
            echo "  Mode: MAPPING (resume)"
        else
            RTAB_ARGS="--delete_db_on_start"
            MEM_PARAM="-p Mem/IncrementalMemory:=true"
            echo "  Mode: MAPPING (fresh)"
        fi ;;
esac

ros2 run rtabmap_slam rtabmap \
    --ros-args \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p map_frame_id:=map \
    -p subscribe_depth:=true \
    -p subscribe_rgb:=true \
    -p subscribe_odom_info:=false \
    -p approx_sync:=true \
    -p wait_for_transform:=0.3 \
    -p queue_size:=30 \
    -p topic_queue_size:=30 \
    -p database_path:=$DB_PATH \
    -p Rtabmap/DetectionRate:="1.0" \
    -p RGBD/LinearUpdate:="0.1" \
    -p RGBD/AngularUpdate:="0.17" \
    -p Vis/FeatureType:=6 \
    -p Kp/DetectorStrategy:=6 \
    -p Vis/MaxFeatures:=600 \
    -p Vis/MinInliers:=8 \
    -p Kp/MaxFeatures:=600 \
    -p Vis/MaxDepth:="5.0" \
    -p Grid/FromDepth:=true \
    -p Grid/MaxObstacleHeight:="0.6" \
    -p Grid/MinObstacleHeight:="0.02" \
    -p Grid/RangeMax:="5.0" \
    -p Grid/CellSize:="0.05" \
    -p Grid/DepthDecimation:=2 \
    -p Reg/Force3DoF:=true \
    -p RGBD/OptimizeFromGraphEnd:=false \
    -p RGBD/ProximityBySpace:=true \
    -p Mem/NotLinkedNodesKept:=false \
    $MEM_PARAM \
    --remap rgb/image:=/camera/camera/color/image_raw \
    --remap rgb/camera_info:=/camera/camera/color/camera_info \
    --remap depth/image:=/camera/camera/aligned_depth_to_color/image_raw \
    --remap grid_map:=/map \
    -- $RTAB_ARGS \
    2>/tmp/slam_rtabmap.log &

RTAB_PID=$!
echo "  Waiting for RTAB-Map (8 s)..."
sleep 8

if ! ps -p $RTAB_PID >/dev/null 2>&1; then
    echo "  ✗ RTAB-Map failed!"
    tail -30 /tmp/slam_rtabmap.log
    exit 1
fi
echo "  ✓ RTAB-Map running (PID $RTAB_PID)"
echo ""

# ── [6/6] NAV2 ────────────────────────────────────────────────────────────
echo "[6/6] Nav2 (waiting 10 s for map to be published)..."
sleep 10

# Check /map is being published before starting Nav2
MAP_FOUND=false
for i in $(seq 1 15); do
    ros2 topic list 2>/dev/null | grep -q "^/map$" && { MAP_FOUND=true; break; }
    echo "  Waiting for /map topic ($i/15)..."
    sleep 2
done

if [ "$MAP_FOUND" = false ]; then
    echo "  ⚠  /map not found yet — Nav2 will retry internally"
    echo "  Tip: rotate camera slowly for 10 s to generate first keyframe"
fi

ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    params_file:=$NAV2_PARAMS \
    2>/tmp/slam_nav2.log &

NAV2_PID=$!
echo "  Waiting for Nav2 (10 s)..."
sleep 10

if ps -p $NAV2_PID >/dev/null 2>&1; then
    echo "  ✓ Nav2 running (PID $NAV2_PID)"
else
    echo "  ✗ Nav2 failed — check: tail /tmp/slam_nav2.log"
    echo "  You can still use RTAB-Map without Nav2 for mapping"
fi
echo ""

# ── Summary ───────────────────────────────────────────────────────────────
echo "========================================="
echo "  ✓✓✓ MINI PC READY ✓✓✓"
echo "========================================="
echo ""
echo "PIDs:"
echo "  odom    $ODOM_PID"
echo "  camera  $CAM_PID"
echo "  vodom   $VODOM_PID"
echo "  rtabmap $RTAB_PID"
echo "  nav2    $NAV2_PID (if started)"
echo ""
echo "KEY TOPICS:"
echo "  /map                     ← occupancy grid (RTAB-Map)"
echo "  /rtabmap/cloud_map       ← 3D point cloud map"
echo "  /odom                    ← dead-reckoning odometry"
echo "  /scan                    ← laser scan (from depth)"
echo "  /cmd_vel                 ← drive commands in"
echo ""
echo "ON LAPTOP:"
echo "  bash slam_laptop.sh"
echo ""
echo "WORKFLOW:"
echo "  1. On laptop run slam_laptop.sh"
echo "  2. Rotate rover camera 360° slowly (~20 s)"
echo "  3. Watch 3D map build in RViz"
echo "  4. Use '2D Nav Goal' tool in RViz to send Nav2 goals"
echo "  5. Ctrl+C here to stop and save map"
echo ""
echo "LOGS:"
echo "  tail -f /tmp/slam_camera.log"
echo "  tail -f /tmp/slam_rtabmap.log"
echo "  tail -f /tmp/slam_nav2.log"
echo ""
echo "Map saved to: $DB_PATH"
echo "Press Ctrl+C to stop"
echo "========================================="

# ── Monitor loop ──────────────────────────────────────────────────────────
TICK=0
while true; do
    sleep 5
    TICK=$((TICK+1))

    for NAME_PID in "RTAB-Map:$RTAB_PID" "Camera:$CAM_PID" "Odom:$ODOM_PID"; do
        NAME="${NAME_PID%%:*}"
        PID="${NAME_PID##*:}"
        if ! ps -p $PID >/dev/null 2>&1; then
            echo ""
            echo "⚠  $NAME (PID $PID) died!"
            echo "   Map saved to: $DB_PATH"
            echo "   Check logs in /tmp/slam_*.log"
            kill 0; wait
            exit 1
        fi
    done

    if [ $((TICK % 12)) -eq 0 ]; then
        NODES=$(ros2 node list 2>/dev/null | grep -c "" || echo "?")
        echo "[$(date +%H:%M:%S)] Running — $NODES nodes"
    fi
done