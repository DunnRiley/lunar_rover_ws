#!/usr/bin/env python3
"""
Complete Teleop for Real Hardware
- Drive robot (w/a/s/d)
- Rotate camera with motor on USB7 (t/g)
- Control actuators on USB4 and USB5 (q/e)
- Emergency stops
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import serial
import time
import threading


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
            print(f"Actuators connected on {self.port1} and {self.port2}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect actuators: {e}")
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


class CameraMotorController:
    """Controls camera rotation motor via serial port"""
    
    def __init__(self, port='/dev/ttyUSB7', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.current_angle = 0.0
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
            print(f"Camera motor connected on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect camera motor: {e}")
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
        """Rotate camera left"""
        if not self.serial:
            return
        
        self._stop_flag = False
        self.serial.rts = False
        self.serial.dtr = True
        self.current_angle += 5.0
        
        time.sleep(duration)
        if not self._stop_flag:
            self.stop()
    
    def rotate_right(self, duration=0.2):
        """Rotate camera right"""
        if not self.serial:
            return
        
        self._stop_flag = False
        self.serial.rts = True
        self.serial.dtr = False
        self.current_angle -= 5.0
        
        time.sleep(duration)
        if not self._stop_flag:
            self.stop()
    
    def center(self):
        """Return camera to center position"""
        if not self.serial:
            return
        
        print(f"Centering camera (current: {self.current_angle:.1f}°)...")
        rotation_needed = -self.current_angle
        
        if abs(rotation_needed) < 2.0:
            print("Camera already centered!")
            self.current_angle = 0.0
            return
        
        if rotation_needed > 0:
            steps = int(abs(rotation_needed) / 5.0)
            for _ in range(steps):
                self.rotate_left(duration=0.2)
                time.sleep(0.1)
        else:
            steps = int(abs(rotation_needed) / 5.0)
            for _ in range(steps):
                self.rotate_right(duration=0.2)
                time.sleep(0.1)
        
        self.current_angle = 0.0
        print("Camera centered")


class CompleteTeleop(Node):
    def __init__(self):
        super().__init__('complete_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Hardware controllers
        self.camera_motor = CameraMotorController('/dev/ttyUSB7')
        self.actuators = ActuatorController('/dev/ttyUSB4', '/dev/ttyUSB5')
        
        # Connect hardware
        self.camera_connected = self.camera_motor.connect()
        self.actuators_connected = self.actuators.connect()
        
        # Fixed speeds
        self.linear_speed = 0.5
        self.angular_speed = 0.8
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Complete Teleop Started')
        self.print_instructions()
        
    def print_instructions(self):
        camera_status = "CONNECTED" if self.camera_connected else "NOT CONNECTED"
        actuator_status = "CONNECTED" if self.actuators_connected else "NOT CONNECTED"
        
        msg = f"""

           ROVER COMPLETE TELEOP CONTROLS               

 ROBOT MOVEMENT (4-Wheel Drive):                       
   W - Forward                                          
   S - Backward                                         
   A - Turn Left                                        
   D - Turn Right                                       
   X - STOP DRIVE                                       
                                                        
 CAMERA ROTATION (USB7): {camera_status:27s} 
   T - Rotate Left                                      
   G - Rotate Right                                     
   Z - Center Camera                                    
   Current Angle: {self.camera_motor.current_angle:5.1f}°                           
                                                        
 ACTUATORS (USB4 & USB5): {actuator_status:25s} 
   Q - Extend Actuators                                 
   E - Retract Actuators                                
   R - STOP ACTUATORS                                   
                                                        
 EMERGENCY:                                             
   SPACE - EMERGENCY STOP ALL                           
                                                        
 INFO:                                                  
   H - Show this help                                   
   CTRL+C - Quit                                        


Speeds: Linear={self.linear_speed:.1f} m/s, Angular={self.angular_speed:.1f} rad/s
Ready for commands!
        """
        print(msg)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def emergency_stop_all(self):
        """Emergency stop everything"""
        self.get_logger().error('EMERGENCY STOP ALL')
        
        # Stop drive
        self.cmd_vel_pub.publish(Twist())
        
        # Stop camera
        if self.camera_connected:
            self.camera_motor.stop()
        
        # Stop actuators
        if self.actuators_connected:
            self.actuators.stop()
        
        print("\n✓ All systems stopped\n")
    
    def run(self):
        try:
            while True:
                key = self.get_key().lower()
                twist = Twist()
                
                # EMERGENCY STOP ALL
                if key == ' ':
                    self.emergency_stop_all()
                
                # ROBOT MOVEMENT
                elif key == 'w':
                    twist.linear.x = self.linear_speed
                    self.get_logger().info('Forward')
                    self.cmd_vel_pub.publish(twist)
                    
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                    self.get_logger().info('Backward')
                    self.cmd_vel_pub.publish(twist)
                    
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                    self.get_logger().info('Turn Left')
                    self.cmd_vel_pub.publish(twist)
                    
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                    self.get_logger().info('Turn Right')
                    self.cmd_vel_pub.publish(twist)
                    
                elif key == 'x':
                    self.get_logger().warn('Stop Drive')
                    self.cmd_vel_pub.publish(twist)
                
                # CAMERA ROTATION
                elif key == 't':
                    if self.camera_connected:
                        self.get_logger().info(f'Camera Left ({self.camera_motor.current_angle:.1f}°)')
                        self.camera_motor.rotate_left()
                    else:
                        self.get_logger().warn('Camera motor not connected!')
                    
                elif key == 'g':
                    if self.camera_connected:
                        self.get_logger().info(f'Camera Right ({self.camera_motor.current_angle:.1f}°)')
                        self.camera_motor.rotate_right()
                    else:
                        self.get_logger().warn('Camera motor not connected!')
                    
                elif key == 'z':
                    if self.camera_connected:
                        self.camera_motor.center()
                    else:
                        self.get_logger().warn('Camera motor not connected!')
                
                # ACTUATORS
                elif key == 'q':
                    if self.actuators_connected:
                        self.get_logger().info('Actuators Extending')
                        self.actuators.extend()
                        # Auto-stop after short duration for safety
                        time.sleep(0.5)
                        self.actuators.stop()
                    else:
                        self.get_logger().warn('Actuators not connected!')
                    
                elif key == 'e':
                    if self.actuators_connected:
                        self.get_logger().info('Actuators Retracting')
                        self.actuators.retract()
                        # Auto-stop after short duration for safety
                        time.sleep(0.5)
                        self.actuators.stop()
                    else:
                        self.get_logger().warn('Actuators not connected!')
                    
                elif key == 'r':
                    if self.actuators_connected:
                        self.get_logger().info('Stop Actuators')
                        self.actuators.stop()
                
                # INFO
                elif key == 'h':
                    self.print_instructions()
                    
                elif key == '\x03':  # Ctrl+C
                    break
                    
                else:
                    if key.isprintable():
                        self.get_logger().warn(f'Unknown key: "{key}". Press H for help.')
                
        except KeyboardInterrupt:
            pass
        finally:
            # Shutdown sequence
            self.get_logger().info('Shutting down...')
            self.emergency_stop_all()
            
            if self.camera_connected:
                self.camera_motor.disconnect()
            if self.actuators_connected:
                self.actuators.disconnect()
            
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info('Teleop shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    teleop = CompleteTeleop()
    try:
        teleop.run()
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()