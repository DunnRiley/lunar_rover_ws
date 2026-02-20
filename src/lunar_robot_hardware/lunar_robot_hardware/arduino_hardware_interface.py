#!/usr/bin/env python3
"""
Arduino Hardware Interface - Matches Existing Arduino Firmware
Protocol: [0xAA][Device][Speed][Direction][0x55]
- Speed: 0-255 (unsigned)
- Direction: 0 or 1 (0=forward, 1=backward)
"""

import serial
import time
import threading
from enum import IntEnum
from typing import Optional
from dataclasses import dataclass


class DeviceID(IntEnum):
    """Device identification bytes - matches your Arduino code"""
    FL_WHEEL = 0x01      # Front Left Wheel
    FR_WHEEL = 0x02      # Front Right Wheel
    BL_WHEEL = 0x03      # Back Left Wheel
    BR_WHEEL = 0x04      # Back Right Wheel
    ACTUATORS = 0x08     # Both actuators
    ACT_LEFT = 0xF7      # Left actuator only
    ACT_RIGHT = 0xD4     # Right actuator only
    STOP_ALL = 0xFF      # Emergency stop all


@dataclass
class ArduinoConfig:
    """Configuration for Arduino connection"""
    port: str = '/dev/ttyACM0'
    baudrate: int = 115200
    timeout: float = 1.0
    max_reconnect_attempts: int = 3
    
    START_BYTE: int = 0xAA
    END_BYTE: int = 0x55


class ArduinoProtocol:
    """Handles the communication protocol"""
    
    @staticmethod
    def encode_command(device_id: int, speed: int, direction: int) -> bytes:
        """
        Encode a command packet matching your Arduino firmware
        
        Args:
            device_id: Device ID from DeviceID enum
            speed: Unsigned speed (0-255)
            direction: 0=forward, 1=backward
        
        Returns:
            5-byte packet [0xAA][Device][Speed][Direction][0x55]
        """
        speed = max(0, min(255, speed))  # Clamp to 0-255
        direction = 1 if direction else 0  # Ensure 0 or 1
        
        packet = bytes([
            ArduinoConfig.START_BYTE,
            device_id,
            speed,
            direction,
            ArduinoConfig.END_BYTE
        ])
        
        return packet


class ArduinoConnection:
    """Manages serial connection to Arduino"""
    
    def __init__(self, config: ArduinoConfig):
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._is_connected = False
        
    def connect(self) -> bool:
        """Establish connection to Arduino"""
        for attempt in range(self.config.max_reconnect_attempts):
            try:
                self.serial = serial.Serial(
                    port=self.config.port,
                    baudrate=self.config.baudrate,
                    timeout=self.config.timeout,
                    write_timeout=self.config.timeout
                )
                
                # Wait for Arduino to reset
                time.sleep(2)
                
                # Flush buffers
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                
                self._is_connected = True
                print(f"✓ Connected to Arduino on {self.config.port}")
                return True
                
            except serial.SerialException as e:
                print(f"Connection attempt {attempt + 1} failed: {e}")
                time.sleep(1)
        
        print(f"✗ Failed to connect after {self.config.max_reconnect_attempts} attempts")
        return False
    
    def send_command(self, device_id: int, speed: int, direction: int) -> bool:
        """Send a command to Arduino"""
        if not self._is_connected or not self.serial:
            return False
        
        with self._lock:
            try:
                packet = ArduinoProtocol.encode_command(device_id, speed, direction)
                self.serial.write(packet)
                return True
            except Exception as e:
                print(f"Error sending command: {e}")
                return False
    
    def close(self):
        """Close the serial connection"""
        if self.serial:
            self.serial.close()
            self._is_connected = False
            print("Arduino connection closed")
    
    @property
    def is_connected(self) -> bool:
        return self._is_connected


class SkidSteerChassis:
    """High-level control for 4-wheel skid-steer rover"""
    
    def __init__(self, connection: ArduinoConnection):
        self.conn = connection
        self.max_speed = 255
        
    def set_wheel(self, device_id: int, speed: int, forward: bool = True):
        """Control individual wheel"""
        direction = 0 if forward else 1
        self.conn.send_command(device_id, abs(speed), direction)
    
    def set_all_wheels(self, fl_speed: int, fr_speed: int, 
                       bl_speed: int, br_speed: int):
        """
        Set all wheels individually
        Positive speed = forward, Negative = backward
        """
        self.set_wheel(DeviceID.FL_WHEEL, abs(fl_speed), fl_speed >= 0)
        self.set_wheel(DeviceID.FR_WHEEL, abs(fr_speed), fr_speed >= 0)
        self.set_wheel(DeviceID.BL_WHEEL, abs(bl_speed), bl_speed >= 0)
        self.set_wheel(DeviceID.BR_WHEEL, abs(br_speed), br_speed >= 0)
    
    def drive(self, linear: float, angular: float):
        """
        Differential drive control
        
        Args:
            linear: Forward speed (-1.0 to 1.0)
            angular: Turn rate (-1.0 to 1.0)
        """
        # Clamp inputs
        linear = max(-1.0, min(1.0, linear))
        angular = max(-1.0, min(1.0, angular))
        
        # Calculate left and right side speeds
        left_speed = linear - angular
        right_speed = linear + angular
        
        # Normalize if either exceeds 1.0
        max_val = max(abs(left_speed), abs(right_speed))
        if max_val > 1.0:
            left_speed /= max_val
            right_speed /= max_val
        
        # Scale to motor speed
        left_motor = int(left_speed * self.max_speed)
        right_motor = int(right_speed * self.max_speed)
        
        # Send to all wheels
        self.set_all_wheels(left_motor, right_motor, left_motor, right_motor)
    
    def stop(self):
        """Stop all drive wheels"""
        self.set_all_wheels(0, 0, 0, 0)
    
    def forward(self, speed: int = 128):
        """Drive forward"""
        self.set_all_wheels(speed, speed, speed, speed)
    
    def backward(self, speed: int = 128):
        """Drive backward"""
        self.set_all_wheels(-speed, -speed, -speed, -speed)
    
    def turn_left(self, speed: int = 128):
        """Pivot turn left"""
        self.set_all_wheels(-speed, speed, -speed, speed)
    
    def turn_right(self, speed: int = 128):
        """Pivot turn right"""
        self.set_all_wheels(speed, -speed, speed, -speed)


class ActuatorController:
    """Control excavation actuators"""
    
    def __init__(self, connection: ArduinoConnection):
        self.conn = connection
        
    def extend(self, speed: int = 200):
        """Extend actuators"""
        self.conn.send_command(DeviceID.ACTUATORS, abs(speed), 0)  # 0 = forward/extend
    
    def retract(self, speed: int = 200):
        """Retract actuators"""
        self.conn.send_command(DeviceID.ACTUATORS, abs(speed), 1)  # 1 = backward/retract
    
    def stop(self):
        """Stop actuators"""
        self.conn.send_command(DeviceID.ACTUATORS, 0, 0)
    
    def extend_left(self, speed: int = 200):
        """Extend left actuator only"""
        self.conn.send_command(DeviceID.ACT_LEFT, abs(speed), 0)
    
    def retract_left(self, speed: int = 200):
        """Retract left actuator only"""
        self.conn.send_command(DeviceID.ACT_LEFT, abs(speed), 1)
    
    def extend_right(self, speed: int = 200):
        """Extend right actuator only"""
        self.conn.send_command(DeviceID.ACT_RIGHT, abs(speed), 0)
    
    def retract_right(self, speed: int = 200):
        """Retract right actuator only"""
        self.conn.send_command(DeviceID.ACT_RIGHT, abs(speed), 1)


class ArduinoRover:
    """Complete rover system interface"""
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200):
        self.config = ArduinoConfig(port=port, baudrate=baudrate)
        self.connection = ArduinoConnection(self.config)
        self.chassis = SkidSteerChassis(self.connection)
        self.actuators = ActuatorController(self.connection)
        self._emergency_stopped = False
    
    def connect(self) -> bool:
        """Connect to Arduino"""
        return self.connection.connect()
    
    def emergency_stop(self):
        """Emergency stop all systems"""
        self._emergency_stopped = True
        self.connection.send_command(DeviceID.STOP_ALL, 0, 0)
        print("🛑 EMERGENCY STOP ACTIVATED")
    
    def resume(self):
        """Clear emergency stop"""
        self._emergency_stopped = False
        print("✓ Emergency stop cleared")
    
    def process_cmd_vel(self, linear_x: float, angular_z: float):
        """
        Process ROS cmd_vel message
        
        Args:
            linear_x: Linear velocity in m/s
            angular_z: Angular velocity in rad/s
        """
        if self._emergency_stopped:
            return
        
        # Normalize to -1.0 to 1.0 range
        # Assuming max linear speed = 0.5 m/s, max angular = 1.0 rad/s
        linear_norm = linear_x / 0.5
        angular_norm = angular_z / 1.0
        
        self.chassis.drive(linear_norm, angular_norm)
    
    def close(self):
        """Shutdown rover safely"""
        self.chassis.stop()
        self.actuators.stop()
        self.connection.close()
    
    def __enter__(self):
        """Context manager entry"""
        if not self.connect():
            raise ConnectionError("Failed to connect to Arduino")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()


def create_arduino_rover(port: str = '/dev/ttyACM0', 
                        baudrate: int = 115200) -> ArduinoRover:
    """
    Factory function to create and connect to rover
    
    Example:
        rover = create_arduino_rover('/dev/ttyACM0')
        rover.connect()
        rover.chassis.forward(speed=100)
        time.sleep(2)
        rover.chassis.stop()
        rover.close()
    """
    rover = ArduinoRover(port, baudrate)
    return rover


# Test code
if __name__ == '__main__':
    import sys
    
    print("Arduino Rover Hardware Interface Test")
    print("=" * 50)
    
    # Detect Arduino port
    import glob
    possible_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    
    if not possible_ports:
        print("No Arduino found!")
        print("Please connect Arduino and try again")
        sys.exit(1)
    
    port = possible_ports[0]
    print(f"Using port: {port}")
    print()
    
    try:
        with create_arduino_rover(port) as rover:
            print("\n1. Testing forward movement...")
            rover.chassis.forward(speed=100)
            time.sleep(2)
            rover.chassis.stop()
            time.sleep(1)
            
            print("2. Testing backward movement...")
            rover.chassis.backward(speed=100)
            time.sleep(2)
            rover.chassis.stop()
            time.sleep(1)
            
            print("3. Testing left turn...")
            rover.chassis.turn_left(speed=80)
            time.sleep(2)
            rover.chassis.stop()
            time.sleep(1)
            
            print("4. Testing right turn...")
            rover.chassis.turn_right(speed=80)
            time.sleep(2)
            rover.chassis.stop()
            time.sleep(1)
            
            print("5. Testing actuator extend...")
            rover.actuators.extend(speed=150)
            time.sleep(2)
            rover.actuators.stop()
            time.sleep(1)
            
            print("6. Testing actuator retract...")
            rover.actuators.retract(speed=150)
            time.sleep(2)
            rover.actuators.stop()
            
            print("\n✓ All tests completed!")
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
