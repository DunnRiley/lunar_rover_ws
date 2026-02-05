#!/usr/bin/env python3
"""
Multi-Waypoint Navigator with RTAB-Map Integration
- Click multiple points in RViz to create waypoint queue
- Executes waypoints sequentially
- Uses RTAB-Map's occupancy grid for obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
import math
from collections import deque
from enum import Enum


class NavigationState(Enum):
    IDLE = 0
    MAPPING = 1
    NAVIGATING = 2
    REACHED_WAYPOINT = 3


class MultiWaypointNavigator(Node):
    def __init__(self):
        super().__init__('multi_waypoint_navigator')
        
        # Parameters
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('lookahead_distance', 0.8)
        
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # State
        self.state = NavigationState.IDLE
        self.waypoint_queue = deque()  # Queue of waypoints to visit
        self.current_waypoint = None
        self.current_pose = None
        self.occupancy_grid = None
        self.waypoint_markers = []
        
        # Subscribers
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/grid_map',
            self.grid_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.path_pub = self.create_publisher(Path, '/waypoint_path', 10)
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.marker_timer = self.create_timer(0.5, self.publish_waypoint_markers)
        
        self.get_logger().info('Multi-Waypoint Navigator Ready!')
        self.get_logger().info('Click points in RViz to add waypoints')
        self.get_logger().info('Press "2D Nav Goal" to start navigation')
        self.print_instructions()
    
    def print_instructions(self):
        """Print usage instructions"""
        instructions = """

          MULTI-WAYPOINT NAVIGATION INSTRUCTIONS            
                                                            
  1. MAPPING PHASE:                                         
     - Drive rover around to build RTAB-Map                 
     - Watch point cloud build up in RViz                   
                                                            
  2. ADD WAYPOINTS:                                         
     - Click "Publish Point" tool in RViz                   
     - Click on map to add waypoints A, B, C...             
     - Waypoints shown as numbered markers                  
                                                            
  3. START NAVIGATION:                                      
     - Click "2D Nav Goal" tool                             
     - Click anywhere to START sequence                     
     - Rover visits waypoints in order                      
                                                            
  4. COMMANDS:                                              
     - Press Space in teleop to emergency stop              
     - Restart node to clear waypoints                      
                                                            
        """
        print(instructions)
    
    def clicked_point_callback(self, msg: PointStamped):
        """Add waypoint from clicked point"""
        waypoint = (msg.point.x, msg.point.y)
        self.waypoint_queue.append(waypoint)
        
        waypoint_num = len(self.waypoint_queue)
        self.get_logger().info(
            f'Added Waypoint {waypoint_num}: ({msg.point.x:.2f}, {msg.point.y:.2f})'
        )
        self.get_logger().info(f'Total waypoints: {len(self.waypoint_queue)}')
        
        self.publish_waypoint_path()
    
    def goal_callback(self, msg: PoseStamped):
        """Start navigation when 2D Nav Goal is set"""
        if len(self.waypoint_queue) == 0:
            self.get_logger().warn('No waypoints set! Click points first.')
            return
        
        self.state = NavigationState.NAVIGATING
        self.current_waypoint = self.waypoint_queue[0]
        
        self.get_logger().info('Starting waypoint navigation!')
        self.get_logger().info(f'Total waypoints: {len(self.waypoint_queue)}')
    
    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
    
    def grid_callback(self, msg: OccupancyGrid):
        """Update occupancy grid from RTAB-Map"""
        self.occupancy_grid = msg
    
    def control_loop(self):
        """Main control loop - navigate to current waypoint"""
        if self.state != NavigationState.NAVIGATING:
            self.cmd_vel_pub.publish(Twist())
            return
        
        if self.current_pose is None or self.current_waypoint is None:
            return
        
        # Get current position
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        goal_x, goal_y = self.current_waypoint
        
        # Calculate distance to current waypoint
        dist = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
        
        # Check if waypoint reached
        if dist < self.goal_tolerance:
            self.waypoint_reached()
            return
        
        # Navigate to waypoint
        twist = self.calculate_velocity(curr_x, curr_y, goal_x, goal_y)
        self.cmd_vel_pub.publish(twist)
    
    def waypoint_reached(self):
        """Handle reaching a waypoint"""
        waypoint_num = len(self.waypoint_queue) - len(self.waypoint_queue) + 1
        self.get_logger().info(f'Reached waypoint {waypoint_num}!')
        
        # Remove completed waypoint
        self.waypoint_queue.popleft()
        
        # Check if more waypoints exist
        if len(self.waypoint_queue) > 0:
            self.current_waypoint = self.waypoint_queue[0]
            remaining = len(self.waypoint_queue)
            self.get_logger().info(f'Moving to next waypoint ({remaining} remaining)')
        else:
            # All waypoints completed
            self.state = NavigationState.IDLE
            self.current_waypoint = None
            self.cmd_vel_pub.publish(Twist())  # Stop
            self.get_logger().info('All waypoints completed!')
            self.get_logger().info('Click new points to add more waypoints')
    
    def calculate_velocity(self, curr_x, curr_y, goal_x, goal_y):
        """Calculate velocity command to reach goal"""
        twist = Twist()
        
        # Calculate heading to goal
        dx = goal_x - curr_x
        dy = goal_y - curr_y
        desired_yaw = math.atan2(dy, dx)
        
        # Get current yaw
        orientation = self.current_pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate yaw error
        yaw_error = desired_yaw - current_yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Pure pursuit control
        if abs(yaw_error) > 0.5:
            # Large heading error - turn in place
            twist.linear.x = self.forward_speed * 0.3
            twist.angular.z = float(max(-self.turn_speed, min(self.turn_speed, yaw_error * 2.0)))
        else:
            # Small heading error - drive forward with correction
            twist.linear.x = self.forward_speed
            twist.angular.z = float(max(-self.turn_speed/2, min(self.turn_speed/2, yaw_error * 1.5)))
        
        return twist
    
    def publish_waypoint_markers(self):
        """Publish visualization markers for waypoints"""
        marker_array = MarkerArray()
        
        # Create markers for queued waypoints
        for i, (x, y) in enumerate(self.waypoint_queue):
            # Waypoint sphere
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color: Current waypoint = green, others = blue
            if i == 0 and self.state == NavigationState.NAVIGATING:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Waypoint number text
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(x)
            text_marker.pose.position.y = float(y)
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = str(i + 1)
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_waypoint_path(self):
        """Publish path connecting all waypoints"""
        if len(self.waypoint_queue) == 0:
            return
        
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in self.waypoint_queue:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    navigator = MultiWaypointNavigator()
    
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