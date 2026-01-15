#!/usr/bin/env python3
"""
Working Navigator - Subscribes to fixed topics with proper QoS settings
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import math
from heapq import heappush, heappop
from enum import Enum


class NavigationMode(Enum):
    IDLE = 0
    NAVIGATING = 1


class WorkingNavigator(Node):
    def __init__(self):
        super().__init__('working_navigator')
        
        # Parameters
        self.declare_parameter('grid_resolution', 0.15)
        self.declare_parameter('planning_range', 8.0)
        self.declare_parameter('obstacle_threshold', 0.2)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('lookahead_distance', 0.8)
        
        self.grid_res = self.get_parameter('grid_resolution').value
        self.planning_range = self.get_parameter('planning_range').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # State
        self.mode = NavigationMode.IDLE
        self.current_pose = None
        self.goal_pose = None
        self.current_path = None
        self.occupancy_grid = None
        self.grid_origin = None
        self.latest_point_cloud = None
        
        # QoS for point cloud (BEST_EFFORT to match publisher)
        pc_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers - Use FIXED topics
        self.pc_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points_fixed',
            self.point_cloud_callback, pc_qos)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_fixed',
            self.odom_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose',
            self.goal_callback, 10)
        
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point',
            self.clicked_point_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/nav_markers', 10)
        
        # Timers
        self.map_timer = self.create_timer(0.5, self.update_occupancy_grid)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.plan_timer = self.create_timer(1.0, self.replan_if_needed)
        
        self.get_logger().info('🚀 Working Navigator Ready!')
        self.get_logger().info('  Subscribing to FIXED topics')
        self.get_logger().info('  Waiting for data...')
    
    def point_cloud_callback(self, msg):
        self.latest_point_cloud = msg
        if not hasattr(self, '_pc_logged'):
            self.get_logger().info(f'✅ Point cloud receiving! Frame: {msg.header.frame_id}')
            self._pc_logged = True
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if not hasattr(self, '_odom_logged'):
            self.get_logger().info(f'✅ Odometry receiving! Frame: {msg.header.frame_id}')
            self._odom_logged = True
    
    def goal_callback(self, msg):
        self.set_goal(msg.pose.position.x, msg.pose.position.y)
    
    def clicked_point_callback(self, msg):
        self.set_goal(msg.point.x, msg.point.y)
    
    def set_goal(self, x, y):
        from geometry_msgs.msg import Pose
        self.goal_pose = Pose()
        self.goal_pose.position.x = float(x)
        self.goal_pose.position.y = float(y)
        self.goal_pose.position.z = 0.0
        self.goal_pose.orientation.w = 1.0
        
        self.mode = NavigationMode.NAVIGATING
        
        if self.current_pose:
            dist = math.sqrt(
                (x - self.current_pose.position.x)**2 + 
                (y - self.current_pose.position.y)**2
            )
            self.get_logger().info(f'🎯 Goal: ({x:.2f}, {y:.2f}), Distance: {dist:.2f}m')
    
    def update_occupancy_grid(self):
        """Create occupancy grid from point cloud"""
        if self.latest_point_cloud is None or self.current_pose is None:
            return
        
        try:
            # Extract points
            points = []
            for p in pc2.read_points(self.latest_point_cloud, 
                                     field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = p
                if abs(x) < self.planning_range and abs(y) < self.planning_range:
                    if z > 0.05 and z < self.obstacle_threshold:
                        points.append([x, y, z])
            
            if len(points) < 5:
                self.create_empty_grid()
                return
            
            points = np.array(points)
            
            # Create grid relative to robot
            x_min = -self.planning_range / 2
            x_max = self.planning_range / 2
            y_min = -self.planning_range / 2
            y_max = self.planning_range / 2
            
            grid_width = int((x_max - x_min) / self.grid_res)
            grid_height = int((y_max - y_min) / self.grid_res)
            
            grid = np.zeros((grid_height, grid_width), dtype=np.int8)
            self.grid_origin = (x_min, y_min)
            
            # Mark obstacles
            for pt in points:
                gx = int((pt[0] - x_min) / self.grid_res)
                gy = int((pt[1] - y_min) / self.grid_res)
                
                if 0 <= gx < grid_width and 0 <= gy < grid_height:
                    grid[gy, gx] = 100
            
            # Inflate obstacles
            kernel_size = int(self.robot_radius / self.grid_res)
            if kernel_size > 0:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                                   (kernel_size*2+1, kernel_size*2+1))
                inflated = cv2.dilate((grid > 50).astype(np.uint8), kernel)
                grid[inflated > 0] = 100
            
            self.occupancy_grid = grid
            self.publish_occupancy_grid(grid)
            
            if not hasattr(self, '_grid_logged'):
                self.get_logger().info(f'✅ Occupancy grid created ({len(points)} obstacles)')
                self._grid_logged = True
            
        except Exception as e:
            self.get_logger().error(f'Grid error: {e}', throttle_duration_sec=5.0)
    
    def create_empty_grid(self):
        grid_size = int(self.planning_range / self.grid_res)
        grid = np.zeros((grid_size, grid_size), dtype=np.int8)
        self.occupancy_grid = grid
        self.grid_origin = (-self.planning_range / 2, -self.planning_range / 2)
        self.publish_occupancy_grid(grid)
    
    def replan_if_needed(self):
        if self.mode == NavigationMode.NAVIGATING and self.goal_pose and self.occupancy_grid is not None and self.current_pose:
            path = self.plan_path()
            if path:
                self.current_path = path
                self.publish_path(path)
    
    def plan_path(self):
        """Simplified path planning - just straight line for now"""
        if not self.current_pose or not self.goal_pose:
            return None
        
        # For now, just create straight line path
        path = [
            (self.current_pose.position.x, self.current_pose.position.y),
            (self.goal_pose.position.x, self.goal_pose.position.y)
        ]
        return path
    
    def control_loop(self):
        """Follow path"""
        twist = Twist()
        
        if self.mode != NavigationMode.NAVIGATING or not self.current_path or not self.current_pose or not self.goal_pose:
            self.cmd_vel_pub.publish(twist)
            return
        
        # Check if goal reached
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        dist_to_goal = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
        
        if dist_to_goal < self.goal_tolerance:
            self.mode = NavigationMode.IDLE
            self.goal_pose = None
            self.current_path = None
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f'🎯 Goal reached!')
            return
        
        # Simple proportional control toward goal
        dx = goal_x - curr_x
        dy = goal_y - curr_y
        desired_yaw = math.atan2(dy, dx)
        
        # Get current yaw
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        yaw_error = desired_yaw - current_yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Control
        if abs(yaw_error) > 0.3:
            twist.angular.z = float(np.clip(yaw_error * 1.5, -self.turn_speed, self.turn_speed))
            twist.linear.x = self.forward_speed * 0.3
        else:
            twist.linear.x = self.forward_speed
            twist.angular.z = float(np.clip(yaw_error, -self.turn_speed/2, self.turn_speed/2))
        
        self.cmd_vel_pub.publish(twist)
    
    def publish_path(self, path_world):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in path_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_occupancy_grid(self, grid):
        if not self.current_pose:
            return
        
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'odom'
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = float(self.grid_res)
        grid_msg.info.width = int(grid.shape[1])
        grid_msg.info.height = int(grid.shape[0])
        grid_msg.info.origin.position.x = float(self.current_pose.position.x + self.grid_origin[0])
        grid_msg.info.origin.position.y = float(self.current_pose.position.y + self.grid_origin[1])
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = [int(x) for x in grid.flatten().tolist()]
        
        self.grid_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = WorkingNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down')
    finally:
        navigator.cmd_vel_pub.publish(Twist())
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()