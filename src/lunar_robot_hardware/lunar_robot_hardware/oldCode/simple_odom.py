#!/usr/bin/env python3
"""
Simple Odometry Publisher
Publishes basic odometry from cmd_vel integration
For use until proper wheel encoders are available
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class SimpleOdometryPublisher(Node):
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.timer = self.create_timer(0.02, self.update_odometry)  # 50 Hz
        
        self.get_logger().info('Simple Odometry Publisher Started')
    
    def cmd_vel_callback(self, msg):
        """Store commanded velocities"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z
    
    def update_odometry(self):
        """Integrate velocity to get position"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Dead reckoning
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert theta to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()