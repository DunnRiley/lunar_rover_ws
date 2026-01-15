#!/usr/bin/env python3
"""
ROS2 Motor Controller Node - Real Hardware Bridge
Subscribes to /cmd_vel and controls real motors
Drop-in replacement for simulation motor controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
import threading
import time

# Import our hardware abstraction layer
from motor_interface import create_chassis_from_ports, SkidSteerChassis


class RealHardwareMotorController(Node):
    """
    ROS2 node that bridges cmd_vel commands to real hardware motors
    Matches the simulation interface exactly
    """
    
    def __init__(self):
        super().__init__('real_hardware_motor_controller')
        
        # ==================== PARAMETERS ====================
        self.declare_parameter('fr_port', '/dev/ttyUSB0')
        self.declare_parameter('fl_port', '/dev/ttyUSB1')
        self.declare_parameter('br_port', '/dev/ttyUSB2')
        self.declare_parameter('bl_port', '/dev/ttyUSB3')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('deadzone_linear', 0.05)
        self.declare_parameter('deadzone_angular', 0.05)
        self.declare_parameter('watchdog_rate', 10.0)  # Hz
        
        # Get parameters
        fr_port = self.get_parameter('fr_port').value
        fl_port = self.get_parameter('fl_port').value
        br_port = self.get_parameter('br_port').value
        bl_port = self.get_parameter('bl_port').value
        baudrate = self.get_parameter('baudrate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.deadzone_linear = self.get_parameter('deadzone_linear').value
        self.deadzone_angular = self.get_parameter('deadzone_angular').value
        watchdog_rate = self.get_parameter('watchdog_rate').value
        
        # ==================== HARDWARE INTERFACE ====================
        self.get_logger().info('Initializing hardware interface...')
        self.chassis = create_chassis_from_ports(
            fr_port, fl_port, br_port, bl_port, baudrate
        )
        
        # Connect to motors
        if not self.chassis.connect_all():
            self.get_logger().error('❌ Failed to connect to all motors!')
            self.get_logger().error('Check USB connections and port mappings')
            raise RuntimeError('Motor connection failed')
        
        # ==================== STATE ====================
        self.last_cmd_vel_time = self.get_clock().now()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.is_emergency_stopped = False
        
        # ==================== SUBSCRIBERS ====================
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
        
        # ==================== PUBLISHERS ====================
        # Publish motor status for monitoring
        self.status_pub = self.create_publisher(
            String,
            '/motor_status',
            10
        )
        
        # Publish joint states (for RViz visualization)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # ==================== TIMERS ====================
        # Watchdog timer - stops motors if no cmd_vel received
        self.watchdog_timer = self.create_timer(
            1.0 / watchdog_rate,
            self.watchdog_callback
        )
        
        # Status publisher timer
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_status
        )
        
        # Joint state publisher (for visualization)
        self.joint_state_timer = self.create_timer(
            0.05,  # 20 Hz
            self.publish_joint_states
        )
        
        self.get_logger().info('✅ Real Hardware Motor Controller Ready!')
        self.get_logger().info('='*60)
        self.get_logger().info('Motor Ports:')
        self.get_logger().info(f'  Front Right: {fr_port}')
        self.get_logger().info(f'  Front Left:  {fl_port}')
        self.get_logger().info(f'  Back Right:  {br_port}')
        self.get_logger().info(f'  Back Left:   {bl_port}')
        self.get_logger().info('='*60)
        self.get_logger().info('Subscribed to: /cmd_vel')
        self.get_logger().info('Publishing to: /motor_status, /joint_states')
        self.get_logger().info('Waiting for commands...')
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Handle incoming cmd_vel commands
        Identical interface to simulation
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
        
        # Send to hardware
        self.chassis.process_cmd_vel(
            self.current_linear,
            self.current_angular,
            self.deadzone_linear,
            self.deadzone_angular
        )
        
        # Log (throttled)
        self.get_logger().info(
            f'cmd_vel: linear={self.current_linear:.2f}, '
            f'angular={self.current_angular:.2f}',
            throttle_duration_sec=1.0
        )
    
    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop commands"""
        if msg.data and not self.is_emergency_stopped:
            self.get_logger().error('🛑 EMERGENCY STOP ACTIVATED!')
            self.chassis.emergency_stop()
            self.is_emergency_stopped = True
        elif not msg.data and self.is_emergency_stopped:
            self.get_logger().info('✅ Emergency stop cleared')
            self.chassis.clear_emergency_stop()
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
                self.chassis.stop()
                self.current_linear = 0.0
                self.current_angular = 0.0
    
    def publish_status(self):
        """Publish motor controller status"""
        status_msg = String()
        
        if self.is_emergency_stopped:
            status = 'EMERGENCY_STOP'
        elif self.chassis.is_fully_connected:
            if self.current_linear != 0.0 or self.current_angular != 0.0:
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
        Since we don't have encoders, publish estimated states
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
        
        # Estimate positions (integrate velocity over time)
        # Without encoders, this is just for visualization
        dt = 0.05  # 20 Hz
        estimated_wheel_velocity = self.current_linear * 10.0  # rad/s estimate
        
        msg.position = [0.0] * 4  # Would need encoder feedback
        msg.velocity = [estimated_wheel_velocity] * 4
        msg.effort = [0.0] * 4
        
        self.joint_state_pub.publish(msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down motor controller...')
        self.chassis.stop()
        self.chassis.disconnect_all()
        self.get_logger().info('✅ Motor controller shutdown complete')
    
    def destroy_node(self):
        """Override to ensure clean shutdown"""
        self.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = RealHardwareMotorController()
        
        # Run the node
        rclpy.spin(controller)
    
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    
    except Exception as e:
        if 'controller' in locals():
            controller.get_logger().error(f'Fatal error: {e}')
        else:
            print(f'Fatal error during initialization: {e}')
    
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()