#!/usr/bin/env python3
"""
Motor Interface - Hardware Abstraction Layer
OOP design for real rover motor control via USB serial

Mirrors the simulation Motor/Chassis classes but controls real hardware.
Designed to be easily upgraded when variable speed control is added.
"""

import serial
import time
from enum import Enum
from typing import Optional, Dict
import threading
from dataclasses import dataclass


class MotorDirection(Enum):
    """Motor direction states"""
    FORWARD = 1
    BACKWARD = -1
    STOP = 0


@dataclass
class MotorConfig:
    """Configuration for a single motor"""
    port: str
    position: str  # 'front_right', 'front_left', etc.
    baudrate: int = 9600
    timeout: float = 1.0
    max_reconnect_attempts: int = 3


class Motor:
    """
    Single motor controller using RTS/DTR serial control
    
    Hardware Interface:
    - RTS LOW + DTR HIGH = Forward
    - RTS HIGH + DTR LOW = Backward  
    - RTS HIGH + DTR HIGH = Stop
    
    Future upgrade path: Replace set_direction() internals when
    PWM or CAN control is added, keeping the same external API.
    """
    
    def __init__(self, config: MotorConfig):
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self._direction = MotorDirection.STOP
        self._speed = 0.0  # Reserved for future variable speed
        self._lock = threading.Lock()
        self._is_connected = False
        
    def connect(self) -> bool:
        """Establish serial connection with retry logic"""
        for attempt in range(self.config.max_reconnect_attempts):
            try:
                self.serial = serial.Serial(
                    port=self.config.port,
                    baudrate=self.config.baudrate,
                    timeout=self.config.timeout
                )
                self.stop()
                self._is_connected = True
                print(f"✓ Connected: {self.config.position} on {self.config.port}")
                return True
                
            except serial.SerialException as e:
                print(f"✗ Attempt {attempt + 1}/{self.config.max_reconnect_attempts} "
                      f"failed for {self.config.position}: {e}")
                time.sleep(0.5)
        
        print(f"✗ FAILED: Could not connect to {self.config.position}")
        return False
    
    def disconnect(self):
        """Safely disconnect motor"""
        with self._lock:
            if self.serial and self.serial.is_open:
                self.stop()
                self.serial.close()
                self._is_connected = False
                print(f"Disconnected: {self.config.position}")
    
    def stop(self):
        """Stop motor movement"""
        with self._lock:
            if self.serial:
                self.serial.rts = True
                self.serial.dtr = True
                self._direction = MotorDirection.STOP
    
    def forward(self):
        """Move motor forward"""
        with self._lock:
            if self.serial:
                self.serial.rts = False  # RTS LOW → ON
                self.serial.dtr = True   # DTR HIGH → OFF
                self._direction = MotorDirection.FORWARD
    
    def backward(self):
        """Move motor backward"""
        with self._lock:
            if self.serial:
                self.serial.rts = True   # RTS HIGH → OFF
                self.serial.dtr = False  # DTR LOW → ON
                self._direction = MotorDirection.BACKWARD
    
    def set_direction(self, direction: MotorDirection):
        """Set motor direction"""
        if direction == MotorDirection.FORWARD:
            self.forward()
        elif direction == MotorDirection.BACKWARD:
            self.backward()
        else:
            self.stop()
    
    def set_speed(self, speed: float):
        """
        Set motor speed (reserved for future PWM/CAN control)
        
        Args:
            speed: -1.0 (full backward) to 1.0 (full forward)
        
        Current behavior: Binary on/off
        Future: Will map to PWM duty cycle or CAN velocity command
        """
        self._speed = max(-1.0, min(1.0, speed))
        
        # Current implementation: binary control
        if speed > 0.05:
            self.forward()
        elif speed < -0.05:
            self.backward()
        else:
            self.stop()
    
    @property
    def is_connected(self) -> bool:
        return self._is_connected
    
    @property
    def current_direction(self) -> MotorDirection:
        return self._direction
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


class SkidSteerChassis:
    """
    4-wheel skid-steer chassis controller
    Mirrors the simulation Chassis class but controls real hardware
    """
    
    def __init__(self, motor_configs: Dict[str, MotorConfig]):
        """
        Initialize chassis with motor configurations
        
        Args:
            motor_configs: Dictionary mapping position to MotorConfig
                          Keys: 'front_right', 'front_left', 'back_right', 'back_left'
        """
        self.motors = {
            pos: Motor(config) for pos, config in motor_configs.items()
        }
        self._emergency_stop = False
        self._watchdog_timeout = 0.5  # seconds
        self._last_command_time = time.time()
        
    def connect_all(self) -> bool:
        """Connect to all motors"""
        print("Connecting to chassis motors...")
        success = True
        for motor in self.motors.values():
            if not motor.connect():
                success = False
        
        if success:
            print("✓ Chassis fully connected!")
        else:
            print("✗ Some motors failed to connect")
        
        return success
    
    def disconnect_all(self):
        """Disconnect all motors"""
        print("Disconnecting chassis...")
        for motor in self.motors.values():
            motor.disconnect()
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all motors"""
        self._emergency_stop = True
        for motor in self.motors.values():
            motor.stop()
        print("🛑 EMERGENCY STOP ACTIVATED")
    
    def clear_emergency_stop(self):
        """Clear emergency stop state"""
        self._emergency_stop = False
        print("✓ Emergency stop cleared")
    
    def check_watchdog(self):
        """Check if commands are still being received"""
        if time.time() - self._last_command_time > self._watchdog_timeout:
            self.stop()
    
    def _update_command_time(self):
        """Update last command timestamp"""
        self._last_command_time = time.time()
    
    # ==================== MOVEMENT COMMANDS ====================
    
    def stop(self):
        """Stop all motors"""
        for motor in self.motors.values():
            motor.stop()
    
    def forward(self):
        """Move forward - all motors forward"""
        if self._emergency_stop:
            return
        self._update_command_time()
        for motor in self.motors.values():
            motor.forward()
    
    def backward(self):
        """Move backward - all motors backward"""
        if self._emergency_stop:
            return
        self._update_command_time()
        for motor in self.motors.values():
            motor.backward()
    
    def turn_left(self):
        """Pivot turn left - left motors backward, right forward"""
        if self._emergency_stop:
            return
        self._update_command_time()
        self.motors['front_left'].backward()
        self.motors['back_left'].backward()
        self.motors['front_right'].forward()
        self.motors['back_right'].forward()
    
    def turn_right(self):
        """Pivot turn right - left motors forward, right backward"""
        if self._emergency_stop:
            return
        self._update_command_time()
        self.motors['front_left'].forward()
        self.motors['back_left'].forward()
        self.motors['front_right'].backward()
        self.motors['back_right'].backward()
    
    def gentle_turn_left(self):
        """Gentle left - right motors on, left stopped"""
        if self._emergency_stop:
            return
        self._update_command_time()
        self.motors['front_left'].stop()
        self.motors['back_left'].stop()
        self.motors['front_right'].forward()
        self.motors['back_right'].forward()
    
    def gentle_turn_right(self):
        """Gentle right - left motors on, right stopped"""
        if self._emergency_stop:
            return
        self._update_command_time()
        self.motors['front_left'].forward()
        self.motors['back_left'].forward()
        self.motors['front_right'].stop()
        self.motors['back_right'].stop()
    
    def process_cmd_vel(self, linear: float, angular: float, 
                       deadzone_linear: float = 0.05,
                       deadzone_angular: float = 0.05):
        """
        Process cmd_vel-style commands (matches simulation interface)
        
        Args:
            linear: Linear velocity (-1.0 to 1.0)
            angular: Angular velocity (-1.0 to 1.0)
            deadzone_linear: Minimum linear value to trigger motion
            deadzone_angular: Minimum angular value to trigger motion
        """
        if self._emergency_stop:
            return
        
        self._update_command_time()
        
        # Apply deadzones
        if abs(linear) < deadzone_linear:
            linear = 0.0
        if abs(angular) < deadzone_angular:
            angular = 0.0
        
        # Stop command
        if abs(linear) < 0.01 and abs(angular) < 0.01:
            self.stop()
            return
        
        # Pure rotation (pivot turn)
        if abs(linear) < 0.01:
            if angular > 0:
                self.turn_left()
            else:
                self.turn_right()
            return
        
        # Pure forward/backward
        if abs(angular) < 0.01:
            if linear > 0:
                self.forward()
            else:
                self.backward()
            return
        
        # Combined motion (forward/backward + turn)
        if linear > 0:
            if angular > 0:
                self.gentle_turn_left()
            else:
                self.gentle_turn_right()
        else:
            # Backward with turn (simplified)
            if angular > 0:
                self.motors['front_left'].stop()
                self.motors['back_left'].stop()
                self.motors['front_right'].backward()
                self.motors['back_right'].backward()
            else:
                self.motors['front_left'].backward()
                self.motors['back_left'].backward()
                self.motors['front_right'].stop()
                self.motors['back_right'].stop()
    
    @property
    def is_fully_connected(self) -> bool:
        """Check if all motors are connected"""
        return all(motor.is_connected for motor in self.motors.values())
    
    def __enter__(self):
        self.connect_all()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect_all()


# ==================== FACTORY FUNCTION ====================

def create_chassis_from_ports(fr_port: str, fl_port: str, 
                              br_port: str, bl_port: str,
                              baudrate: int = 9600) -> SkidSteerChassis:
    """
    Factory function to create chassis from port strings
    
    Args:
        fr_port: Front right motor port (e.g., '/dev/ttyUSB0')
        fl_port: Front left motor port
        br_port: Back right motor port
        bl_port: Back left motor port
        baudrate: Serial communication baud rate
    
    Returns:
        Configured SkidSteerChassis instance
    """
    motor_configs = {
        'front_right': MotorConfig(fr_port, 'front_right', baudrate),
        'front_left': MotorConfig(fl_port, 'front_left', baudrate),
        'back_right': MotorConfig(br_port, 'back_right', baudrate),
        'back_left': MotorConfig(bl_port, 'back_left', baudrate),
    }
    
    return SkidSteerChassis(motor_configs)


if __name__ == '__main__':
    # Test the motor interface
    print("="*60)
    print("MOTOR INTERFACE TEST")
    print("="*60)
    
    chassis = create_chassis_from_ports(
        '/dev/ttyUSB0',  # FR
        '/dev/ttyUSB1',  # FL
        '/dev/ttyUSB2',  # BR
        '/dev/ttyUSB3'   # BL
    )
    
    try:
        with chassis:
            if chassis.is_fully_connected:
                print("\n✓ All motors connected! Running test sequence...\n")
                
                print("Test 1: Forward for 2 seconds")
                chassis.forward()
                time.sleep(2)
                
                print("Test 2: Stop")
                chassis.stop()
                time.sleep(1)
                
                print("Test 3: Turn left for 2 seconds")
                chassis.turn_left()
                time.sleep(2)
                
                print("Test 4: Stop")
                chassis.stop()
                time.sleep(1)
                
                print("\n✓ Test complete!")
            else:
                print("\n✗ Not all motors connected - test aborted")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        chassis.emergency_stop()