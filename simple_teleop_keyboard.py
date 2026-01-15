#!/usr/bin/env python3
"""
Simplified Teleop for Real Hardware
- Drive robot (w/a/s/d)
- Rotate camera with motor on USB7 (t/g)
- Center camera (z)
- No speed control (hardware limitation for now)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys
import termios
import tty
import serial
import time
import threading


class CameraMotorController:
    """Controls camera rotation motor via serial port"""
    
    def __init__(self, port='/dev/ttyUSB7', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.current_angle = 0.0  # Track approximate angle
        self.center_position = 0.0
        self._stop_flag = False
        
    def connect(self):
        """Connect to camera motor"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.stop()
            print(f"✓ Camera motor connected on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect camera motor: {e}")
            return False
    
    def disconnect(self):
        """Disconnect camera motor"""
        if self.serial and self.serial.is_open:
            self.stop()
            self.serial.close()
    
    def stop(self):
        """Stop camera motor"""
        if self.serial:
            self.serial.rts = True
            self.serial.dtr = True
            self._stop_flag = False
    
    def rotate_left(self, duration=0.2):
        """Rotate camera left for specified duration"""
        if not self.serial:
            return
        
        self._stop_flag = False
        self.serial.rts = False  # RTS LOW → ON
        self.serial.dtr = True   # DTR HIGH → OFF
        
        # Approximate angle tracking (rough estimate)
        self.current_angle += 5.0  # degrees per step
        
        time.sleep(duration)
        if not self._stop_flag:
            self.stop()
    
    def rotate_right(self, duration=0.2):
        """Rotate camera right for specified duration"""
        if not self.serial:
            return
        
        self._stop_flag = False
        self.serial.rts = True   # RTS HIGH → OFF
        self.serial.dtr = False  # DTR LOW → ON
        
        # Approximate angle tracking
        self.current_angle -= 5.0  # degrees per step
        
        time.sleep(duration)
        if not self._stop_flag:
            self.stop()
    
    def center(self):
        """Return camera to center position (approximate)"""
        if not self.serial:
            return
        
        print(f"Centering camera (current: {self.current_angle:.1f}°)...")
        
        # Calculate rotation needed
        rotation_needed = -self.current_angle
        
        if abs(rotation_needed) < 2.0:
            print("Camera already centered!")
            self.current_angle = 0.0
            return
        
        # Determine direction and duration
        if rotation_needed > 0:
            # Need to rotate left
            steps = int(abs(rotation_needed) / 5.0)
            for _ in range(steps):
                self.rotate_left(duration=0.2)
                time.sleep(0.1)
        else:
            # Need to rotate right
            steps = int(abs(rotation_needed) / 5.0)
            for _ in range(steps):
                self.rotate_right(duration=0.2)
                time.sleep(0.1)
        
        self.current_angle = 0.0
        print("✓ Camera centered")


class SimpleTeleop(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Camera motor controller
        self.camera_motor = CameraMotorController('/dev/ttyUSB7')
        self.camera_connected = self.camera_motor.connect()
        
        # Fixed speeds (no control yet due to hardware)
        self.linear_speed = 0.5
        self.angular_speed = 0.8
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Simple Teleop Started')
        self.print_instructions()
        
    def print_instructions(self):
        camera_status = "✓ CONNECTED" if self.camera_connected else "✗ NOT CONNECTED"
        msg = f"""
╔══════════════════════════════════════════════════════╗
║          LUNAR ROVER TELEOP CONTROLS                 ║
╚══════════════════════════════════════════════════════╝

ROBOT MOVEMENT:
  w : Forward
  s : Backward
  a : Turn Left
  d : Turn Right
  x : STOP

CAMERA ROTATION (Motor on USB7):
  t : Rotate Left
  g : Rotate Right
  z : Center Camera
  
  Camera Status: {camera_status}
  Current Angle: {self.camera_motor.current_angle:.1f}°

INFO:
  h : Show this help
  
CTRL+C to quit

════════════════════════════════════════════════════════
        """
        print(msg)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        try:
            while True:
                key = self.get_key().lower()
                twist = Twist()
                
                # ROBOT MOVEMENT
                if key == 'w':
                    twist.linear.x = self.linear_speed
                    self.get_logger().info('↑ Forward')
                    self.cmd_vel_pub.publish(twist)
                    
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                    self.get_logger().info('↓ Backward')
                    self.cmd_vel_pub.publish(twist)
                    
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                    self.get_logger().info('← Turn Left')
                    self.cmd_vel_pub.publish(twist)
                    
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                    self.get_logger().info('→ Turn Right')
                    self.cmd_vel_pub.publish(twist)
                    
                elif key == 'x':
                    self.get_logger().warn('⏹ STOP')
                    self.cmd_vel_pub.publish(twist)
                
                # CAMERA ROTATION
                elif key == 't':
                    if self.camera_connected:
                        self.get_logger().info(f'📷 ← Camera Left ({self.camera_motor.current_angle:.1f}°)')
                        self.camera_motor.rotate_left()
                    else:
                        self.get_logger().warn('Camera motor not connected!')
                    
                elif key == 'g':
                    if self.camera_connected:
                        self.get_logger().info(f'📷 → Camera Right ({self.camera_motor.current_angle:.1f}°)')
                        self.camera_motor.rotate_right()
                    else:
                        self.get_logger().warn('Camera motor not connected!')
                    
                elif key == 'z':
                    if self.camera_connected:
                        self.camera_motor.center()
                    else:
                        self.get_logger().warn('Camera motor not connected!')
                
                # INFO
                elif key == 'h':
                    self.print_instructions()
                    
                elif key == '\x03':  # Ctrl+C
                    break
                    
                else:
                    if key.isprintable():
                        self.get_logger().warn(f'Unknown key: "{key}". Press h for help.')
                
        except KeyboardInterrupt:
            pass
        finally:
            # Stop everything
            self.cmd_vel_pub.publish(Twist())
            if self.camera_connected:
                self.camera_motor.disconnect()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info('Teleop shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    teleop = SimpleTeleop()
    try:
        teleop.run()
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()