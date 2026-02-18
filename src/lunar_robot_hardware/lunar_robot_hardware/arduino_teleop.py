#!/usr/bin/env python3
"""
Hold-to-Drive Teleop for Arduino-based Lunar Rover
Drives only while keys are held down, stops when released
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys
import termios
import tty
import select
import threading


class HoldToDriveTeleop(Node):
    def __init__(self):
        super().__init__('arduino_teleop_hold')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Try to connect directly to Arduino for actuator control
        try:
            import sys
            import os
            # Add the module path
            sys.path.append(os.path.expanduser('~/lunar_rover_ws/src/lunar_robot_hardware/lunar_robot_hardware'))
            from arduino_hardware_interface import create_arduino_rover
            
            self.get_logger().info('Attempting to connect to Arduino for actuator control...')
            self.arduino = create_arduino_rover('/dev/ttyACM0', 115200)
            self.has_arduino = True
            self.get_logger().info('Arduino connection established')
        except Exception as e:
            self.get_logger().warn(f'Could not connect to Arduino: {e}')
            self.get_logger().warn('Actuator controls (Q/E/R) will not work')
            self.arduino = None
            self.has_arduino = False
        
        # Speed settings (m/s and rad/s)
        self.linear_speed = 0.5
        self.angular_speed = 0.8
        self.actuator_speed = 100  # -127 to 127
        
        # Current key states
        self.active_keys = set()
        self.key_lock = threading.Lock()
        
        # Terminal settings
        self.settings = None
        
        # Control loop timer (50Hz for smooth control)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.print_instructions()
    
    def print_instructions(self):
        print("\n" + "="*60)
        print("HOLD-TO-DRIVE TELEOP - Arduino Lunar Rover")
        print("="*60)
        print("\nDRIVE CONTROLS (hold key down):")
        print("  W - Drive forward")
        print("  S - Drive backward")
        print("  A - Turn left")
        print("  D - Turn right")
        print("\nACTUATOR CONTROLS (hold key down):")
        print("  Q - Extend actuators")
        print("  E - Retract actuators")
        print("\nSAFETY:")
        print("  SPACE - Emergency stop ALL systems")
        print("\nOTHER:")
        print("  H - Show this help")
        print("  Ctrl+C - Quit")
        print("\nMotors will STOP when you release the keys")
        print("="*60 + "\n")
    
    def control_loop(self):
        """Main control loop - runs at 50Hz"""
        with self.key_lock:
            current_keys = self.active_keys.copy()
        
        # Initialize twist message
        twist = Twist()
        
        # Process drive keys
        if 'w' in current_keys:
            twist.linear.x = self.linear_speed
        elif 's' in current_keys:
            twist.linear.x = -self.linear_speed
        
        if 'a' in current_keys:
            twist.angular.z = self.angular_speed
        elif 'd' in current_keys:
            twist.angular.z = -self.angular_speed
        
        # Publish drive command
        self.cmd_vel_pub.publish(twist)
        
        # Process actuator keys (direct Arduino control)
        if self.has_arduino:
            if 'q' in current_keys:
                # Extend actuators
                try:
                    self.arduino.actuators.extend(self.actuator_speed)
                except:
                    pass
            elif 'e' in current_keys:
                # Retract actuators
                try:
                    self.arduino.actuators.retract(self.actuator_speed)
                except:
                    pass
            else:
                # Stop actuators when no key pressed
                try:
                    self.arduino.actuators.stop()
                except:
                    pass
    
    def process_key(self, key):
        """Process individual key press"""
        if key == ' ':  # Space - Emergency stop
            self.emergency_stop()
        elif key == 'h':  # Help
            self.print_instructions()
        elif key in ['w', 's', 'a', 'd', 'q', 'e']:
            # Add to active keys
            with self.key_lock:
                self.active_keys.add(key)
    
    def clear_key(self, key):
        """Clear key from active set when released"""
        with self.key_lock:
            self.active_keys.discard(key)
    
    def emergency_stop(self):
        """Emergency stop all systems"""
        self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
        
        # Stop drive motors
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Stop actuators
        if self.has_arduino:
            try:
                self.arduino.actuators.stop()
            except:
                pass
        
        # Publish emergency stop message
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)
        
        # Clear all active keys
        with self.key_lock:
            self.active_keys.clear()
        
        print("\n*** ALL SYSTEMS STOPPED ***\n")
    
    def run(self):
        """Main run loop - handles keyboard input"""
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
            print("Ready! Press keys to drive (release to stop)...\n")
            
            while rclpy.ok():
                # Check if key is available
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    # Check for Ctrl+C
                    if ord(key) == 3:  # Ctrl+C
                        break
                    
                    self.process_key(key)
                
                # Spin ROS2 node
                rclpy.spin_once(self, timeout_sec=0)
                
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown"""
        print("\n\nShutting down...")
        
        # Stop all motors
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Stop actuators
        if self.has_arduino:
            try:
                self.arduino.actuators.stop()
                self.arduino.close()
            except:
                pass
        
        # Restore terminal settings
        if self.settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
        print("Teleop stopped safely.")


def main(args=None):
    rclpy.init(args=args)
    
    teleop = HoldToDriveTeleop()
    
    try:
        teleop.run()
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()