#!/usr/bin/env python3
"""
ROS2 Motor Controller Node - Arduino Hardware Bridge
Uses the modern Arduino hardware interface
Compatible with point-click navigation and teleop
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
import time

# Import our new Arduino hardware interface
from arduino_hardware_interface import ArduinoRover, create_arduino_rover


class ArduinoMotorController(Node):
    """
    ROS2 node that bridges cmd_vel commands to Arduino hardware
    Drop-in replacement for old motor controller
    """
    
    def __init__(self):
        super().__init__('arduino_motor_controller')
        
        # ========== PARAMETERS ==========
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('deadzone_linear', 0.05)
        self.declare_parameter('deadzone_angular', 0.05)
        self.declare_parameter('max_motor_speed', 127)
        self.declare_parameter('watchdog_rate', 10.0)  # Hz
        
        # Get parameters
        arduino_port = self.get_parameter('arduino_port').value
        baudrate = self.get_parameter('baudrate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.deadzone_linear = self.get_parameter('deadzone_linear').value
        self.deadzone_angular = self.get_parameter('deadzone_angular').value
        self.max_motor_speed = self.get_parameter('max_motor_speed').value
        watchdog_rate = self.get_parameter('watchdog_rate').value
        
        # ========== HARDWARE INTERFACE ==========
        self.get_logger().info(f'Connecting to Arduino on {arduino_port}...')
        self.rover = create_arduino_rover(port=arduino_port, baudrate=baudrate)
        
        if not self.rover.is_connected:
            self.get_logger().error('Failed to connect to Arduino!')
            self.get_logger().error('Check USB connection and port mapping')
            raise RuntimeError('Arduino connection failed')
        
        # ========== STATE ==========
        self.last_cmd_vel_time = self.get_clock().now()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.is_emergency_stopped = False
        
        # ========== SUBSCRIBERS ==========
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # ========== PUBLISHERS ==========
        self.status_pub = self.create_publisher(
            String,
            '/motor_status',
            10
        )
        
        # Publish joint states for RViz visualization
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # ========== TIMERS ==========
        # Watchdog timer - stops motors if no cmd_vel received
        self.watchdog_timer = self.create_timer(
            1.0 / watchdog_rate,
            self.watchdog_callback
        )
        
        # Status publisher
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_status
        )
        
        # Joint state publisher (for RViz)
        self.joint_state_timer = self.create_timer(
            0.05,  # 20 Hz
            self.publish_joint_states
        )
        
        self.get_logger().info('='*60)
        self.get_logger().info('Arduino Motor Controller Ready!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Arduino Port: {arduino_port}')
        self.get_logger().info(f'Baudrate: {baudrate}')
        self.get_logger().info(f'Max Motor Speed: {self.max_motor_speed}')
        self.get_logger().info('='*60)
        self.get_logger().info('Subscribed to: /cmd_vel, /emergency_stop')
        self.get_logger().info('Publishing to: /motor_status, /joint_states')
        self.get_logger().info('Waiting for commands...')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Handle incoming cmd_vel commands
        Compatible with point-click navigation and teleop
        """
        if self.is_emergency_stopped:
            self.get_logger().warn(
                '⚠️ Emergency stop active - ignoring cmd_vel',
                throttle_duration_sec=2.0
            )
            return
        
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Extract velocities
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        
        # Send to Arduino using the new interface
        self.rover.chassis.process_cmd_vel(
            self.current_linear,
            self.current_angular,
            self.deadzone_linear,
            self.deadzone_angular,
            self.max_motor_speed
        )
        
        # Log (throttled)
        if abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01:
            self.get_logger().info(
                f'cmd_vel: linear={self.current_linear:.2f}, '
                f'angular={self.current_angular:.2f}',
                throttle_duration_sec=1.0
            )
    
    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop commands"""
        if msg.data and not self.is_emergency_stopped:
            self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED!')
            self.rover.emergency_stop_all()
            self.is_emergency_stopped = True
        elif not msg.data and self.is_emergency_stopped:
            self.get_logger().info('Emergency stop cleared')
            self.rover.chassis.clear_emergency_stop()
            self.is_emergency_stopped = False
    
    def watchdog_callback(self):
        """
        Watchdog timer - stops motors if no cmd_vel received
        Safety feature to prevent runaway
        """
        if self.is_emergency_stopped:
            return
        
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        
        if elapsed > self.cmd_vel_timeout:
            # No command received recently - stop motors
            if self.current_linear != 0.0 or self.current_angular != 0.0:
                self.get_logger().warn(
                    f'⚠️ No cmd_vel for {elapsed:.2f}s - stopping motors',
                    throttle_duration_sec=2.0
                )
                self.rover.chassis.stop()
                self.current_linear = 0.0
                self.current_angular = 0.0
    
    def publish_status(self):
        """Publish motor controller status"""
        status_msg = String()
        
        if self.is_emergency_stopped:
            status = 'EMERGENCY_STOP'
        elif self.rover.is_connected:
            if abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01:
                status = 'MOVING'
            else:
                status = 'IDLE'
        else:
            status = 'ERROR_DISCONNECTED'
        
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def publish_joint_states(self):
        """
        Publish joint states for RViz visualization
        Estimated states based on cmd_vel (no encoders yet)
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Wheel joint names (must match URDF)
        msg.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        # Estimate wheel velocities from cmd_vel
        # This is just for visualization - real implementation needs encoders
        wheel_radius = 0.1  # meters (adjust to match your rover)
        estimated_left_vel = (self.current_linear - self.current_angular) / wheel_radius
        estimated_right_vel = (self.current_linear + self.current_angular) / wheel_radius
        
        msg.position = [0.0] * 4  # Would need encoder feedback
        msg.velocity = [
            estimated_left_vel,   # front left
            estimated_right_vel,  # front right
            estimated_left_vel,   # rear left
            estimated_right_vel   # rear right
        ]
        msg.effort = [0.0] * 4
        
        self.joint_state_pub.publish(msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down motor controller...')
        self.rover.chassis.stop()
        self.rover.actuators.stop()
        self.rover.disconnect()
        self.get_logger().info('Motor controller shutdown complete')
    
    def destroy_node(self):
        """Override to ensure clean shutdown"""
        self.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = ArduinoMotorController()
        
        # Run the node
        rclpy.spin(controller)
    
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    
    except Exception as e:
        if 'controller' in locals():
            controller.get_logger().error(f'Fatal error: {e}')
        else:
            print(f'Fatal error during initialization: {e}')
            print('\nTroubleshooting:')
            print('1. Check Arduino is connected: ls /dev/ttyACM*')
            print('2. Check Arduino has correct firmware loaded')
            print('3. Check USB cable and connection')
    
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()