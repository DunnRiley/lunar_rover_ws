#!/usr/bin/env python3
"""
Multi-Waypoint Navigator with Course Change Support
- Click multiple waypoints in RViz (even outside current view)
- Navigate sequentially with Nav2
- Ability to change course mid-navigation
- Works with persistent SLAM map
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from enum import Enum
from collections import deque
import math


class NavigationState(Enum):
    IDLE = 0
    NAVIGATING = 1
    PAUSED = 2


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # State
        self.state = NavigationState.IDLE
        self.waypoint_queue = deque()
        
        # Action client for Nav2
        self._navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
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
        
        self.cancel_sub = self.create_subscription(
            Bool,
            '/cancel_navigation',
            self.cancel_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )
        
        self.path_pub = self.create_publisher(
            Path,
            '/waypoint_path',
            10
        )
        
        # Timers
        self.marker_timer = self.create_timer(0.5, self.publish_markers)
        
        self.get_logger().info('='*60)
        self.get_logger().info('  Multi-Waypoint Navigator Ready!')
        self.get_logger().info('='*60)
        self.get_logger().info('')
        self.get_logger().info('  Click "Publish Point" in RViz to add waypoints')
        self.get_logger().info('  Click "2D Goal Pose" to START navigation')
        self.get_logger().info('  Publish to /cancel_navigation to stop')
        self.get_logger().info('')
        self.get_logger().info('  You can add waypoints ANYWHERE on the map,')
        self.get_logger().info('  even 180° behind the robot!')
        self.get_logger().info('='*60)
    
    def clicked_point_callback(self, msg: PointStamped):
        """Add waypoint from clicked point"""
        # Create pose from point
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'  # Use map frame
        pose.pose.position = msg.point
        pose.pose.orientation.w = 1.0  # Default orientation
        
        self.waypoint_queue.append(pose)
        
        waypoint_num = len(self.waypoint_queue)
        self.get_logger().info(
            f'✓ Added Waypoint #{waypoint_num}: '
            f'({msg.point.x:.2f}, {msg.point.y:.2f}) in {msg.header.frame_id}'
        )
        self.get_logger().info(f'  Total waypoints in queue: {len(self.waypoint_queue)}')
        
        self.publish_waypoint_path()
    
    def goal_callback(self, msg: PoseStamped):
        """Start navigation when 2D Goal Pose is clicked"""
        if len(self.waypoint_queue) == 0:
            self.get_logger().warn('⚠️  No waypoints set! Click points first with "Publish Point"')
            return
        
        if self.is_navigating:
            self.get_logger().warn('⚠️  Already navigating! Publishing to /cancel_navigation to stop first')
            return
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'🚀 Starting navigation through {len(self.waypoint_queue)} waypoints!')
        self.get_logger().info('='*60)
        
        self.state = NavigationState.NAVIGATING
        self.navigate_to_next_waypoint()
    
    def cancel_callback(self, msg: Bool):
        """Cancel current navigation"""
        if msg.data:
            self.get_logger().warn('🛑 Navigation cancelled by user')
            self.cancel_navigation()
            self.state = NavigationState.IDLE
            self.is_navigating = False
    
    def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in the queue"""
        if len(self.waypoint_queue) == 0:
            self.get_logger().info('🎉 All waypoints completed!')
            self.state = NavigationState.IDLE
            self.is_navigating = False
            return
        
        # Get next waypoint
        self.current_goal = self.waypoint_queue[0]
        
        remaining = len(self.waypoint_queue)
        self.get_logger().info(
            f'📍 Navigating to waypoint ({remaining} remaining): '
            f'({self.current_goal.pose.position.x:.2f}, '
            f'{self.current_goal.pose.position.y:.2f})'
        )
        
        # Send goal to Nav2
        self.send_goal(self.current_goal)
    
    def send_goal(self, pose: PoseStamped):
        """Send navigation goal to Nav2"""
        # Wait for action server
        self.get_logger().info('Waiting for Nav2 action server...')
        self._navigate_to_pose_client.wait_for_server()
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Send goal
        self.is_navigating = True
        self._send_goal_future = self._navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by Nav2!')
            self.navigate_to_next_waypoint()  # Try next waypoint
            return
        
        self.get_logger().info('✓ Goal accepted by Nav2, navigating...')
        
        # Wait for result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        
        # Log progress (throttled)
        self.get_logger().info(
            f'Distance remaining: {distance:.2f}m',
            throttle_duration_sec=2.0
        )
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Waypoint reached!')
            
            # Remove completed waypoint
            self.waypoint_queue.popleft()
            
            # Navigate to next
            self.navigate_to_next_waypoint()
        
        elif status == 5:  # CANCELED
            self.get_logger().warn('Navigation was cancelled')
            self.is_navigating = False
        
        else:  # FAILED
            self.get_logger().error(f'❌ Navigation failed with status: {status}')
            
            # Remove failed waypoint and try next
            self.waypoint_queue.popleft()
            self.navigate_to_next_waypoint()
    
    def cancel_navigation(self):
        """Cancel current navigation goal"""
        if self.is_navigating:
            # Cancel goal
            self._navigate_to_pose_client._cancel_goal_async(self._send_goal_future)
            self.is_navigating = False
            self.get_logger().info('Navigation cancelled')
    
    def publish_markers(self):
        """Publish visualization markers for waypoints"""
        marker_array = MarkerArray()
        
        for i, pose in enumerate(self.waypoint_queue):
            # Waypoint sphere
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color: current = green, others = blue
            if i == 0 and self.is_navigating:
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
            text = Marker()
            text.header = marker.header
            text.ns = 'waypoint_labels'
            text.id = i + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pose.pose.position.x
            text.pose.position.y = pose.pose.position.y
            text.pose.position.z = pose.pose.position.z + 0.4
            text.scale.z = 0.3
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"#{i+1}"
            
            marker_array.markers.append(text)
        
        self.marker_pub.publish(marker_array)
    
    def publish_waypoint_path(self):
        """Publish path connecting waypoints"""
        if len(self.waypoint_queue) == 0:
            return
        
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for pose in self.waypoint_queue:
            path.poses.append(pose)
        
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down...')
    finally:
        if navigator.is_navigating:
            navigator.cancel_navigation()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()