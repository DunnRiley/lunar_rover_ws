#!/usr/bin/env python3
"""
Arduino Hardware Interface - Modern OOP Approach
Communicates with Arduino Mega 2560 via serial protocol
Protocol: [0xAA][Device][Signed_Speed][0x55]
"""

import serial
import time
import threading
from enum import IntEnum
from typing import Optional, Dict, List
from dataclasses import dataclass
import struct


class DeviceID(IntEnum):
    """Device identification bytes"""
    FL_WHEEL = 0x01      # Front Left Wheel
    FR_WHEEL = 0x02      # Front Right Wheel
    BL_WHEEL = 0x03      # Back Left Wheel
    BR_WHEEL = 0x04      # Back Right Wheel
    ACTUATORS = 0x08     # Both actuators together
    SERVO1 = 0x10
    SERVO2 = 0x11


@dataclass
class ArduinoConfig:
    """Configuration for Arduino connection"""
    port: str = '/dev/ttyACM0'  # Arduino Mega typically shows as ACM
    baudrate: int = 115200       # Match Arduino's Serial.begin(115200)
    timeout: float = 1.0
    max_reconnect_attempts: int = 3
    
    # Protocol bytes
    START_BYTE: int = 0xAA
    END_BYTE: int = 0x55


class ArduinoProtocol:
    """Handles the communication protocol with Arduino"""
    
    @staticmethod
    def encode_command(device_id: int, speed: int) -> bytes:
        """
        Encode a command packet
        
        Args:
            device_id: Device ID from DeviceID enum
            speed: Signed speed value (-127 to +127)
        
        Returns:
            4-byte command packet [START][DEVICE][SPEED][END]
        """
        # Ensure speed is in valid range
        speed = max(-127, min(127, speed))
        
        # Convert to signed byte
        speed_byte = speed & 0xFF
        
        packet = bytes([
            ArduinoConfig.START_BYTE,
            device_id,
            speed_byte,
            ArduinoConfig.END_BYTE
        ])
        
        return packet
    
    @staticmethod
    def validate_packet(packet: bytes) -> bool:
        """Validate a received packet"""
        if len(packet) != 4:
            return False
        if packet[0] != ArduinoConfig.START_BYTE:
            return False
        if packet[3] != ArduinoConfig.END_BYTE:
            return False
        return True


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
                
                # Wait for Arduino to reset (it resets on serial connection)
                time.sleep(2.0)
                
                # Flush any startup messages
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                
                self._is_connected = True
                print(f"✓ Connected to Arduino on {self.config.port}")
                return True
                
            except serial.SerialException as e:
                print(f"✗ Attempt {attempt + 1}/{self.config.max_reconnect_attempts} "
                      f"failed: {e}")
                time.sleep(0.5)
        
        print(f"✗ Failed to connect to Arduino on {self.config.port}")
        return False
    
    def disconnect(self):
        """Safely disconnect"""
        with self._lock:
            if self.serial and self.serial.is_open:
                self.serial.close()
                self._is_connected = False
                print("Disconnected from Arduino")
    
    def send_command(self, device_id: int, speed: int) -> bool:
        """
        Send a command to Arduino
        
        Args:
            device_id: Device to control
            speed: Speed value (-127 to +127)
        
        Returns:
            True if sent successfully
        """
        if not self._is_connected or not self.serial:
            return False
        
        packet = ArduinoProtocol.encode_command(device_id, speed)
        
        with self._lock:
            try:
                self.serial.write(packet)
                return True
            except serial.SerialException as e:
                print(f"✗ Send error: {e}")
                return False
    
    @property
    def is_connected(self) -> bool:
        return self._is_connected and self.serial and self.serial.is_open


class SkidSteerChassis:
    """
    High-level interface for 4-wheel skid-steer chassis
    Handles differential drive kinematics
    """
    
    def __init__(self, connection: ArduinoConnection):
        self.conn = connection
        self._emergency_stop = False
        
    def emergency_stop(self):
        """Emergency stop - immediately halt all motors"""
        self._emergency_stop = True
        self.stop()
        print("🚨 EMERGENCY STOP ACTIVATED")
    
    def clear_emergency_stop(self):
        """Clear emergency stop state"""
        self._emergency_stop = False
        print("✓ Emergency stop cleared")
    
    def stop(self):
        """Stop all drive motors"""
        for device in [DeviceID.FL_WHEEL, DeviceID.FR_WHEEL, 
                      DeviceID.BL_WHEEL, DeviceID.BR_WHEEL]:
            self.conn.send_command(device, 0)
    
    def forward(self, speed: int = 100):
        """Move forward - all wheels forward"""
        if self._emergency_stop:
            return
        
        for device in [DeviceID.FL_WHEEL, DeviceID.FR_WHEEL,
                      DeviceID.BL_WHEEL, DeviceID.BR_WHEEL]:
            self.conn.send_command(device, speed)
    
    def backward(self, speed: int = 100):
        """Move backward - all wheels backward"""
        if self._emergency_stop:
            return
        
        for device in [DeviceID.FL_WHEEL, DeviceID.FR_WHEEL,
                      DeviceID.BL_WHEEL, DeviceID.BR_WHEEL]:
            self.conn.send_command(device, -speed)
    
    def turn_left(self, speed: int = 80):
        """Pivot turn left - left wheels backward, right forward"""
        if self._emergency_stop:
            return
        
        self.conn.send_command(DeviceID.FL_WHEEL, -speed)
        self.conn.send_command(DeviceID.BL_WHEEL, -speed)
        self.conn.send_command(DeviceID.FR_WHEEL, speed)
        self.conn.send_command(DeviceID.BR_WHEEL, speed)
    
    def turn_right(self, speed: int = 80):
        """Pivot turn right - left wheels forward, right backward"""
        if self._emergency_stop:
            return
        
        self.conn.send_command(DeviceID.FL_WHEEL, speed)
        self.conn.send_command(DeviceID.BL_WHEEL, speed)
        self.conn.send_command(DeviceID.FR_WHEEL, -speed)
        self.conn.send_command(DeviceID.BR_WHEEL, -speed)
    
    def process_cmd_vel(self, linear: float, angular: float,
                       deadzone_linear: float = 0.05,
                       deadzone_angular: float = 0.05,
                       max_speed: int = 127):
        """
        Process ROS cmd_vel-style commands
        
        Args:
            linear: Linear velocity (-1.0 to 1.0)
            angular: Angular velocity (-1.0 to 1.0)
            deadzone_linear: Deadzone for linear velocity
            deadzone_angular: Deadzone for angular velocity
            max_speed: Maximum motor speed (0-127)
        """
        if self._emergency_stop:
            return
        
        # Apply deadzones
        if abs(linear) < deadzone_linear:
            linear = 0.0
        if abs(angular) < deadzone_angular:
            angular = 0.0
        
        # Stop command
        if abs(linear) < 0.01 and abs(angular) < 0.01:
            self.stop()
            return
        
        # Convert to motor speeds using differential drive
        # Left side speed
        left_speed = linear - angular
        # Right side speed
        right_speed = linear + angular
        
        # Normalize to max speed
        max_val = max(abs(left_speed), abs(right_speed))
        if max_val > 1.0:
            left_speed /= max_val
            right_speed /= max_val
        
        # Convert to motor values
        left_motor = int(left_speed * max_speed)
        right_motor = int(right_speed * max_speed)
        
        # Send to motors
        self.conn.send_command(DeviceID.FL_WHEEL, left_motor)
        self.conn.send_command(DeviceID.BL_WHEEL, left_motor)
        self.conn.send_command(DeviceID.FR_WHEEL, right_motor)
        self.conn.send_command(DeviceID.BR_WHEEL, right_motor)


class ActuatorController:
    """
    Controls excavation actuators
    """
    
    def __init__(self, connection: ArduinoConnection):
        self.conn = connection
        
    def stop(self):
        """Stop actuators"""
        self.conn.send_command(DeviceID.ACTUATORS, 0)
    
    def extend(self, speed: int = 100):
        """Extend actuators"""
        self.conn.send_command(DeviceID.ACTUATORS, speed)
    
    def retract(self, speed: int = 100):
        """Retract actuators"""
        self.conn.send_command(DeviceID.ACTUATORS, -speed)


class ArduinoRover:
    """
    Complete rover system with chassis and actuators
    Main interface for high-level control
    """
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200):
        """
        Initialize rover
        
        Args:
            port: Serial port for Arduino (default /dev/ttyACM0)
            baudrate: Serial baudrate (must match Arduino code)
        """
        self.config = ArduinoConfig(port=port, baudrate=baudrate)
        self.connection = ArduinoConnection(self.config)
        self.chassis = SkidSteerChassis(self.connection)
        self.actuators = ActuatorController(self.connection)
        
    def connect(self) -> bool:
        """Connect to Arduino"""
        return self.connection.connect()
    
    def disconnect(self):
        """Disconnect from Arduino"""
        # Safety: stop everything before disconnecting
        self.chassis.stop()
        self.actuators.stop()
        self.connection.disconnect()
    
    @property
    def is_connected(self) -> bool:
        """Check if connected"""
        return self.connection.is_connected
    
    def emergency_stop_all(self):
        """Emergency stop everything"""
        self.chassis.emergency_stop()
        self.actuators.stop()
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


# Factory function for easy setup
def create_arduino_rover(port: str = '/dev/ttyACM0',
                        baudrate: int = 115200) -> ArduinoRover:
    """
    Factory function to create and connect to rover
    
    Args:
        port: Serial port (default /dev/ttyACM0 for Arduino Mega)
        baudrate: Must match Arduino's Serial.begin() value
    
    Returns:
        Connected ArduinoRover instance
    
    Example:
        rover = create_arduino_rover()
        if rover.is_connected:
            rover.chassis.forward()
    """
    rover = ArduinoRover(port, baudrate)
    rover.connect()
    return rover


# ========== TEST CODE ==========
if __name__ == '__main__':
    print("="*70)
    print("  Arduino Rover Hardware Test")
    print("="*70)
    print("\nThis will test the Arduino communication")
    print("Make sure Arduino is connected and loaded with the rover firmware")
    print()
    
    # Check for available serial ports
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    
    if ports:
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device} - {p.description}")
        print()
    else:
        print("⚠️  No serial ports found!")
        print("Make sure Arduino is connected via USB")
        exit(1)
    
    # Try to connect
    port = input("Enter Arduino port [/dev/ttyACM0]: ").strip() or '/dev/ttyACM0'
    
    print(f"\nConnecting to {port}...")
    
    with create_arduino_rover(port=port) as rover:
        if not rover.is_connected:
            print("✗ Connection failed!")
            exit(1)
        
        print("✓ Connected!")
        print("\nRunning test sequence...\n")
        
        try:
            # Test 1: Forward
            print("Test 1: Forward (2 seconds)")
            rover.chassis.forward(speed=80)
            time.sleep(2)
            
            # Test 2: Stop
            print("Test 2: Stop")
            rover.chassis.stop()
            time.sleep(1)
            
            # Test 3: Backward
            print("Test 3: Backward (2 seconds)")
            rover.chassis.backward(speed=80)
            time.sleep(2)
            
            # Test 4: Stop
            print("Test 4: Stop")
            rover.chassis.stop()
            time.sleep(1)
            
            # Test 5: Turn left
            print("Test 5: Turn left (2 seconds)")
            rover.chassis.turn_left(speed=60)
            time.sleep(2)
            
            # Test 6: Stop
            print("Test 6: Stop")
            rover.chassis.stop()
            time.sleep(1)
            
            # Test 7: Turn right
            print("Test 7: Turn right (2 seconds)")
            rover.chassis.turn_right(speed=60)
            time.sleep(2)
            
            # Test 8: Stop
            print("Test 8: Stop")
            rover.chassis.stop()
            time.sleep(1)
            
            # Test 9: Actuators extend
            print("Test 9: Actuators extend (1 second)")
            rover.actuators.extend(speed=80)
            time.sleep(1)
            
            # Test 10: Actuators stop
            print("Test 10: Actuators stop")
            rover.actuators.stop()
            time.sleep(1)
            
            # Test 11: Actuators retract
            print("Test 11: Actuators retract (1 second)")
            rover.actuators.retract(speed=80)
            time.sleep(1)
            
            # Test 12: Final stop
            print("Test 12: Final stop")
            rover.actuators.stop()
            rover.chassis.stop()
            
            print("\n✓ Test sequence complete!")
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Test interrupted by user")
            rover.emergency_stop_all()
        
        except Exception as e:
            print(f"\n\n✗ Error during test: {e}")
            rover.emergency_stop_all()
    
    print("\n✓ Test complete. Arduino disconnected safely.")