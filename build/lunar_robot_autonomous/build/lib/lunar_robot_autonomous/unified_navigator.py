#!/usr/bin/env python3
"""
IMPROVED Unified Navigator - Fixed frames and stopping behavior
- Correct frame transformations
- Proper goal detection and stopping
- Better obstacle detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped, Twist, Point, PointStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
from heapq import heappush, heappop
from enum import Enum


class NavigationMode(Enum):
    IDLE = 0
    NAVIGATING = 1
    BEACON_TRACK = 2


class ImprovedNavigator(Node):
    def __init__(self):
        super().__init__('improved_navigator')
        
        # Parameters
        self.declare_parameter('grid_resolution', 0.15)
        self.declare_parameter('planning_range', 8.0)
        self.declare_parameter('obstacle_threshold', 0.2)  # Lower = more sensitive
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
        
        self.bridge = CvBridge()
        
        # Subscribers - CORRECTED topic names
        self.pc_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points_fixed',
            self.point_cloud_callback, 10)
        
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
        
        self.get_logger().info('🚀 Improved Navigator Ready!')
        self.get_logger().info('  Grid resolution: %.2f m' % self.grid_res)
        self.get_logger().info('  Planning range: %.1f m' % self.planning_range)
        self.get_logger().info('  Goal tolerance: %.2f m' % self.goal_tolerance)
    
    # ==================== SENSOR CALLBACKS ====================
    
    def point_cloud_callback(self, msg):
        self.latest_point_cloud = msg
        if not hasattr(self, '_pc_logged'):
            self.get_logger().info('✅ Point cloud receiving from: %s' % msg.header.frame_id)
            self._pc_logged = True
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if not hasattr(self, '_odom_logged'):
            self.get_logger().info('✅ Odometry receiving!')
            self._odom_logged = True
    
    def goal_callback(self, msg):
        """2D Goal Pose from RViz"""
        self.set_goal(msg.pose.position.x, msg.pose.position.y)
    
    def clicked_point_callback(self, msg):
        """Publish Point from RViz"""
        self.set_goal(msg.point.x, msg.point.y)
    
    def set_goal(self, x, y):
        """Set navigation goal in odom frame"""
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
            self.get_logger().info(f'🎯 New goal: ({x:.2f}, {y:.2f}), Distance: {dist:.2f}m')
        else:
            self.get_logger().warn('⚠️ Goal set but no odometry yet!')
    
    # ==================== OCCUPANCY GRID ====================
    
    def update_occupancy_grid(self):
        if self.latest_point_cloud is None or self.current_pose is None:
            return
        
        try:
            # Extract points from point cloud
            points = []
            for p in pc2.read_points(self.latest_point_cloud, 
                                     field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = p
                # Filter points within planning range
                if abs(x) < self.planning_range and abs(y) < self.planning_range:
                    # IMPORTANT: Obstacles are points ABOVE ground level
                    if z > 0.05 and z < self.obstacle_threshold:
                        points.append([x, y, z])
            
            if len(points) < 5:
                # Create empty grid if no obstacles
                self.create_empty_grid()
                return
            
            points = np.array(points)
            
            # Transform points from camera frame to odom frame
            # The point cloud is in camera_depth_optical_frame
            # We need to transform to odom frame using current robot pose
            transformed_points = self.transform_points_to_odom(points)
            
            # Create occupancy grid
            grid, grid_origin = self.create_occupancy_grid_from_points(transformed_points)
            self.occupancy_grid = grid
            self.grid_origin = grid_origin
            
            # Publish occupancy grid
            self.publish_occupancy_grid(grid, grid_origin)
            
            if not hasattr(self, '_grid_created'):
                self.get_logger().info(f'✅ Occupancy grid created with {len(points)} obstacles')
                self._grid_created = True
            
        except Exception as e:
            self.get_logger().error(f'Grid error: {e}', throttle_duration_sec=5.0)
    
    def transform_points_to_odom(self, points):
        """Transform points from camera frame to odom frame using robot pose"""
        if self.current_pose is None:
            return points
        
        # Get robot position and orientation
        x_robot = self.current_pose.position.x
        y_robot = self.current_pose.position.y
        
        # Get yaw from quaternion
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Transform each point
        transformed = []
        for pt in points:
            # Point in camera frame (x forward, y left, z up)
            x_cam, y_cam, z_cam = pt
            
            # Rotate by robot yaw
            x_rotated = x_cam * math.cos(yaw) - y_cam * math.sin(yaw)
            y_rotated = x_cam * math.sin(yaw) + y_cam * math.cos(yaw)
            
            # Translate by robot position
            x_odom = x_rotated + x_robot
            y_odom = y_rotated + y_robot
            
            transformed.append([x_odom, y_odom, z_cam])
        
        return np.array(transformed) if transformed else np.array([[]])
    
    def create_occupancy_grid_from_points(self, points):
        """Create occupancy grid from obstacle points in odom frame"""
        if len(points) == 0:
            return self.create_empty_grid()
        
        # Grid centered around robot
        if self.current_pose:
            x_min = self.current_pose.position.x - self.planning_range / 2
            x_max = self.current_pose.position.x + self.planning_range / 2
            y_min = self.current_pose.position.y - self.planning_range / 2
            y_max = self.current_pose.position.y + self.planning_range / 2
        else:
            x_min = -self.planning_range / 2
            x_max = self.planning_range / 2
            y_min = -self.planning_range / 2
            y_max = self.planning_range / 2
        
        grid_width = int((x_max - x_min) / self.grid_res)
        grid_height = int((y_max - y_min) / self.grid_res)
        
        grid = np.zeros((grid_height, grid_width), dtype=np.int8)
        grid_origin = (x_min, y_min)
        
        # Mark obstacle cells
        for pt in points:
            gx = int((pt[0] - x_min) / self.grid_res)
            gy = int((pt[1] - y_min) / self.grid_res)
            
            if 0 <= gx < grid_width and 0 <= gy < grid_height:
                grid[gy, gx] = 100
        
        # Inflate obstacles by robot radius
        kernel_size = int(self.robot_radius / self.grid_res)
        if kernel_size > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
                                               (kernel_size*2+1, kernel_size*2+1))
            inflated = cv2.dilate((grid > 50).astype(np.uint8), kernel)
            grid[inflated > 0] = 100
        
        return grid, grid_origin
    
    def create_empty_grid(self):
        """Create empty grid when no obstacles detected"""
        if self.current_pose:
            x_min = self.current_pose.position.x - self.planning_range / 2
            y_min = self.current_pose.position.y - self.planning_range / 2
        else:
            x_min = -self.planning_range / 2
            y_min = -self.planning_range / 2
        
        grid_width = int(self.planning_range / self.grid_res)
        grid_height = int(self.planning_range / self.grid_res)
        
        grid = np.zeros((grid_height, grid_width), dtype=np.int8)
        self.occupancy_grid = grid
        self.grid_origin = (x_min, y_min)
        self.publish_occupancy_grid(grid, (x_min, y_min))
    
    # ==================== PATH PLANNING ====================
    
    def replan_if_needed(self):
        """Replan periodically if navigating"""
        if self.mode == NavigationMode.NAVIGATING and self.goal_pose is not None:
            if self.occupancy_grid is not None and self.current_pose is not None:
                path = self.plan_path()
                if path:
                    self.current_path = path
                    self.publish_path(path)
    
    def plan_path(self):
        if self.occupancy_grid is None or self.current_pose is None:
            return None
        
        start_grid = self.world_to_grid(
            self.current_pose.position.x, 
            self.current_pose.position.y
        )
        goal_grid = self.world_to_grid(
            self.goal_pose.position.x, 
            self.goal_pose.position.y
        )
        
        if not self.is_valid_cell(goal_grid):
            self.get_logger().warn('⚠️ Goal blocked!', throttle_duration_sec=2.0)
            return None
        
        path_grid = self.astar_search(start_grid, goal_grid)
        if path_grid is None:
            return None
        
        path_world = [self.grid_to_world(gx, gy) for gx, gy in path_grid]
        return path_world
    
    def astar_search(self, start, goal):
        """A* pathfinding"""
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        directions = [(1,0), (0,1), (-1,0), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]
        
        iterations = 0
        max_iterations = 5000
        
        while open_set and iterations < max_iterations:
            iterations += 1
            current = heappop(open_set)[1]
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return list(reversed(path))
            
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid_cell(neighbor):
                    continue
                
                tentative_g = g_score[current] + math.sqrt(dx**2 + dy**2)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heappush(open_set, (f_score, neighbor))
        
        return None
    
    # ==================== CONTROL LOOP ====================
    
    def control_loop(self):
        """Follow planned path with proper stopping"""
        twist = Twist()
        
        # Always stop if not navigating
        if self.mode != NavigationMode.NAVIGATING:
            self.cmd_vel_pub.publish(twist)
            return
        
        # Need all data to navigate
        if self.current_path is None or self.current_pose is None or self.goal_pose is None:
            self.cmd_vel_pub.publish(twist)
            return
        
        # Check if goal reached
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y
        
        dist_to_goal = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
        
        # CRITICAL: Stop when goal reached
        if dist_to_goal < self.goal_tolerance:
            self.mode = NavigationMode.IDLE
            self.goal_pose = None
            self.current_path = None
            self.cmd_vel_pub.publish(twist)  # Send stop command
            self.get_logger().info(f'🎯 Goal reached! (distance: {dist_to_goal:.3f}m)')
            return
        
        # Find lookahead point
        target = None
        for wx, wy in self.current_path:
            dist = math.sqrt((wx - curr_x)**2 + (wy - curr_y)**2)
            if dist >= self.lookahead_distance:
                target = (float(wx), float(wy))
                break
        
        if target is None:
            target = (float(self.current_path[-1][0]), float(self.current_path[-1][1]))
        
        # Calculate heading error
        orientation = self.current_pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        dx = target[0] - curr_x
        dy = target[1] - curr_y
        desired_yaw = math.atan2(dy, dx)
        
        yaw_error = desired_yaw - current_yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Pure pursuit control
        if abs(yaw_error) > 0.5:
            twist.linear.x = self.forward_speed * 0.3
            twist.angular.z = float(np.clip(yaw_error * 2.0, -self.turn_speed, self.turn_speed))
        else:
            twist.linear.x = self.forward_speed
            twist.angular.z = float(np.clip(yaw_error * 1.5, -self.turn_speed/2, self.turn_speed/2))
        
        self.cmd_vel_pub.publish(twist)
    
    # ==================== HELPER FUNCTIONS ====================
    
    def is_valid_cell(self, cell):
        gx, gy = cell
        if gx < 0 or gx >= self.occupancy_grid.shape[1]:
            return False
        if gy < 0 or gy >= self.occupancy_grid.shape[0]:
            return False
        return self.occupancy_grid[gy, gx] < 50
    
    def world_to_grid(self, x, y):
        gx = int((x - self.grid_origin[0]) / self.grid_res)
        gy = int((y - self.grid_origin[1]) / self.grid_res)
        return (gx, gy)
    
    def grid_to_world(self, gx, gy):
        x = float(gx * self.grid_res + self.grid_origin[0] + self.grid_res/2)
        y = float(gy * self.grid_res + self.grid_origin[1] + self.grid_res/2)
        return (x, y)
    
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
    
    def publish_occupancy_grid(self, grid, grid_origin):
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'odom'
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = float(self.grid_res)
        grid_msg.info.width = int(grid.shape[1])
        grid_msg.info.height = int(grid.shape[0])
        grid_msg.info.origin.position.x = float(grid_origin[0])
        grid_msg.info.origin.position.y = float(grid_origin[1])
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = [int(x) for x in grid.flatten().tolist()]
        
        self.grid_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = ImprovedNavigator()
    
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