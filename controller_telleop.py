#!/usr/bin/env python3
"""
Gamepad Controller Teleop for Lunar Rover
- Left joystick: Drive robot (forward/back, turn left/right)
- Y button: Extend actuators (only while held)
- X button: Retract actuators (only while held)
- Motors only move while input is active
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import serial
import time


class ActuatorController:
    """Controls actuator motors via serial ports"""
    
    def __init__(self, port1='/dev/ttyUSB4', port2='/dev/ttyUSB5', baudrate=9600):
        self.port1 = port1
        self.port2 = port2
        self.baudrate = baudrate
        self.serial1 = None
        self.serial2 = None
        
    def connect(self):
        """Connect to both actuator motors"""
        try:
            self.serial1 = serial.Serial(
                port=self.port1,
                baudrate=self.baudrate,
                timeout=1
            )
            self.serial2 = serial.Serial(
                port=self.port2,
                baudrate=self.baudrate,
                timeout=1
            )
            self.stop()
            print(f"✓ Actuators connected on {self.port1} and {self.port2}")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect actuators: {e}")
            return False
    
    def disconnect(self):
        """Disconnect actuator motors"""
        if self.serial1 and self.serial1.is_open:
            self.stop()
            self.serial1.close()
        if self.serial2 and self.serial2.is_open:
            self.serial2.close()
    
    def stop(self):
        """Stop both actuator motors"""
        if self.serial1:
            self.serial1.rts = True
            self.serial1.dtr = True
        if self.serial2:
            self.serial2.rts = True
            self.serial2.dtr = True
    
    def extend(self):
        """Extend actuators (forward)"""
        if self.serial1 and self.serial2:
            self.serial1.rts = False  # RTS LOW → ON
            self.serial1.dtr = True   # DTR HIGH → OFF
            self.serial2.rts = False
            self.serial2.dtr = True
    
    def retract(self):
        """Retract actuators (backward)"""
        if self.serial1 and self.serial2:
            self.serial1.rts = True   # RTS HIGH → OFF
            self.serial1.dtr = False  # DTR LOW → ON
            self.serial2.rts = True
            self.serial2.dtr = False


class GamepadTeleop(Node):
    """
    Gamepad controller for rover
    
    Controller mapping (Xbox/PS4 style):
    - Left stick vertical (axis 1): Forward/backward
    - Left stick horizontal (axis 0): Turn left/right
    - Y button (button 3): Extend actuators
    - X button (button 2): Retract actuators
    - Start button (button 7): Emergency stop
    """
    
    def __init__(self):
        super().__init__('gamepad_teleop')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('actuator_port_1', '/dev/ttyUSB4')
        self.declare_parameter('actuator_port_2', '/dev/ttyUSB5')
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Actuator controller
        port1 = self.get_parameter('actuator_port_1').value
        port2 = self.get_parameter('actuator_port_2').value
        self.actuators = ActuatorController(port1, port2)
        self.actuators_connected = self.actuators.connect()
        
        # State
        self.last_joy_time = self.get_clock().now()
        self.emergency_stopped = False
        
        # Timer to stop motors if no input (safety)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        
        self.get_logger().info('╔════════════════════════════════════════════╗')
        self.get_logger().info('║     GAMEPAD TELEOP READY                   ║')
        self.get_logger().info('╠════════════════════════════════════════════╣')
        self.get_logger().info('║ Left Stick: Drive (forward/back/turn)      ║')
        self.get_logger().info('║ Y Button: Extend Actuators (hold)          ║')
        self.get_logger().info('║ X Button: Retract Actuators (hold)         ║')
        self.get_logger().info('║ Start Button: Emergency Stop               ║')
        self.get_logger().info('╠════════════════════════════════════════════╣')
        actuator_status = '✓ CONNECTED' if self.actuators_connected else '✗ NOT CONNECTED'
        self.get_logger().info(f'║ Actuators: {actuator_status:30s} ║')
        self.get_logger().info('╚════════════════════════════════════════════╝')
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def joy_callback(self, msg):
        """Handle gamepad input - motors only move while input is active"""
        
        self.last_joy_time = self.get_clock().now()
        
        # Check for emergency stop (Start button)
        if len(msg.buttons) > 7 and msg.buttons[7] == 1:
            if not self.emergency_stopped:
                self.emergency_stopped = True
                self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED 🚨')
                self.stop_all()
            return
        
        # If emergency stopped, don't process other inputs
        if self.emergency_stopped:
            return
        
        # === DRIVE CONTROL (Left Stick) ===
        # Axis 1: Forward/backward (inverted because up is negative)
        # Axis 0: Left/right turning
        linear = 0.0
        angular = 0.0
        
        if len(msg.axes) > 1:
            linear = -self.apply_deadzone(msg.axes[1]) * self.max_linear
        
        if len(msg.axes) > 0:
            angular = self.apply_deadzone(msg.axes[0]) * self.max_angular
        
        # Publish drive command
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        
        # Log drive commands (throttled)
        if abs(linear) > 0.01 or abs(angular) > 0.01:
            self.get_logger().info(
                f'🚗 Drive: linear={linear:.2f} m/s, angular={angular:.2f} rad/s',
                throttle_duration_sec=0.5
            )
        
        # === ACTUATOR CONTROL (Buttons) ===
        if self.actuators_connected and len(msg.buttons) > 3:
            y_button = msg.buttons[3]  # Y button
            x_button = msg.buttons[2]  # X button
            
            if y_button == 1:
                # Y button held - extend actuators
                self.actuators.extend()
                self.get_logger().info('🔧 Actuators: EXTENDING', throttle_duration_sec=0.5)
            elif x_button == 1:
                # X button held - retract actuators
                self.actuators.retract()
                self.get_logger().info('🔧 Actuators: RETRACTING', throttle_duration_sec=0.5)
            else:
                # No button pressed - stop actuators
                self.actuators.stop()
    
    def watchdog_callback(self):
        """
        Safety watchdog - stop everything if no input for too long
        This ensures motors stop if controller disconnects
        """
        if self.emergency_stopped:
            return
        
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        
        if elapsed > 0.5:  # 500ms timeout
            # No input for too long - stop everything
            self.cmd_vel_pub.publish(Twist())
            if self.actuators_connected:
                self.actuators.stop()
    
    def stop_all(self):
        """Emergency stop all motors"""
        # Stop drive
        self.cmd_vel_pub.publish(Twist())
        
        # Stop actuators
        if self.actuators_connected:
            self.actuators.stop()
        
        self.get_logger().warn('✓ All motors stopped')
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down gamepad teleop...')
        self.stop_all()
        if self.actuators_connected:
            self.actuators.disconnect()
        self.get_logger().info('✓ Shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    
    # Check if joy node is running
    print("\n" + "="*50)
    print("GAMEPAD TELEOP STARTING")
    print("="*50)
    print("\nIMPORTANT: Make sure joy_node is running!")
    print("In another terminal, run:")
    print("  ros2 run joy joy_node")
    print("\nOr add to your launcher.")
    print("="*50 + "\n")
    
    teleop = GamepadTeleop()
    
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        teleop.get_logger().info('Interrupted by user')
    finally:
        teleop.shutdown()
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()