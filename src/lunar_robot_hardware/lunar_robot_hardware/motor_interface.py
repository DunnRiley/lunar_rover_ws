#!/usr/bin/env python3
"""
Motor Interface - Hardware Abstraction Layer
OOP design for real rover motor control via USB serial
NOW INCLUDES: 4 drive motors + 2 auxiliary motors
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
    position: str  # 'front_right', 'front_left', 'aux_1', etc.
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
    """
    
    def __init__(self, config: MotorConfig):
        self.config = config
        self.serial: Optional[serial.Serial] = None
        self._direction = MotorDirection.STOP
        self._speed = 0.0
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


class AuxiliaryMotorPair:
    """
    Pair of auxiliary motors that move together
    Used for things like excavation, collection mechanisms, etc.
    """
    
    def __init__(self, motor1: Motor, motor2: Motor, name: str = "Auxiliary"):
        """
        Initialize auxiliary motor pair
        
        Args:
            motor1: First motor
            motor2: Second motor
            name: Descriptive name for the motor pair
        """
        self.motor1 = motor1
        self.motor2 = motor2
        self.name = name
        
    def connect(self) -> bool:
        """Connect both motors"""
        m1_ok = self.motor1.connect()
        m2_ok = self.motor2.connect()
        return m1_ok and m2_ok
    
    def disconnect(self):
        """Disconnect both motors"""
        self.motor1.disconnect()
        self.motor2.disconnect()
    
    def forward(self):
        """Move both motors forward"""
        self.motor1.forward()
        self.motor2.forward()
    
    def backward(self):
        """Move both motors backward"""
        self.motor1.backward()
        self.motor2.backward()
    
    def stop(self):
        """Stop both motors"""
        self.motor1.stop()
        self.motor2.stop()
    
    @property
    def is_connected(self) -> bool:
        return self.motor1.is_connected and self.motor2.is_connected


class SkidSteerChassis:
    """
    4-wheel skid-steer chassis controller
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
        self._watchdog_timeout = 0.5
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
    
    def _update_command_time(self):
        """Update last command timestamp"""
        self._last_command_time = time.time()
    
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
    
    def process_cmd_vel(self, linear: float, angular: float, 
                       deadzone_linear: float = 0.05,
                       deadzone_angular: float = 0.05):
        """Process cmd_vel-style commands"""
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
        
        # Pure rotation
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
        
        # Combined motion
        if linear > 0:
            if angular > 0:
                self.motors['front_left'].stop()
                self.motors['back_left'].stop()
                self.motors['front_right'].forward()
                self.motors['back_right'].forward()
            else:
                self.motors['front_left'].forward()
                self.motors['back_left'].forward()
                self.motors['front_right'].stop()
                self.motors['back_right'].stop()
        else:
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


class CompleteRover:
    """
    Complete rover system with drive chassis + auxiliary motors
    """
    
    def __init__(self, chassis: SkidSteerChassis, aux_motors: AuxiliaryMotorPair):
        """
        Initialize complete rover
        
        Args:
            chassis: SkidSteerChassis for driving
            aux_motors: AuxiliaryMotorPair for auxiliary functions
        """
        self.chassis = chassis
        self.aux_motors = aux_motors
        
    def connect_all(self) -> bool:
        """Connect to all motors"""
        chassis_ok = self.chassis.connect_all()
        aux_ok = self.aux_motors.connect()
        return chassis_ok and aux_ok
    
    def disconnect_all(self):
        """Disconnect all motors"""
        self.chassis.disconnect_all()
        self.aux_motors.disconnect()
    
    def emergency_stop(self):
        """Emergency stop everything"""
        self.chassis.emergency_stop()
        self.aux_motors.stop()
    
    @property
    def is_fully_connected(self) -> bool:
        return self.chassis.is_fully_connected and self.aux_motors.is_connected


# ==================== FACTORY FUNCTIONS ====================

def create_chassis_from_ports(fr_port: str, fl_port: str, 
                              br_port: str, bl_port: str,
                              baudrate: int = 9600) -> SkidSteerChassis:
    """Create chassis from port strings"""
    motor_configs = {
        'front_right': MotorConfig(fr_port, 'front_right', baudrate),
        'front_left': MotorConfig(fl_port, 'front_left', baudrate),
        'back_right': MotorConfig(br_port, 'back_right', baudrate),
        'back_left': MotorConfig(bl_port, 'back_left', baudrate),
    }
    return SkidSteerChassis(motor_configs)


def create_aux_motors_from_ports(port1: str, port2: str,
                                 baudrate: int = 9600,
                                 name: str = "Auxiliary") -> AuxiliaryMotorPair:
    """Create auxiliary motor pair from port strings"""
    motor1 = Motor(MotorConfig(port1, f'{name}_1', baudrate))
    motor2 = Motor(MotorConfig(port2, f'{name}_2', baudrate))
    return AuxiliaryMotorPair(motor1, motor2, name)


def create_complete_rover(fr_port: str, fl_port: str, br_port: str, bl_port: str,
                         aux1_port: str, aux2_port: str,
                         baudrate: int = 9600) -> CompleteRover:
    """
    Create complete rover with all 6 motors
    
    Args:
        fr_port: Front right drive motor
        fl_port: Front left drive motor
        br_port: Back right drive motor
        bl_port: Back left drive motor
        aux1_port: Auxiliary motor 1
        aux2_port: Auxiliary motor 2
        baudrate: Serial communication rate
    """
    chassis = create_chassis_from_ports(fr_port, fl_port, br_port, bl_port, baudrate)
    aux_motors = create_aux_motors_from_ports(aux1_port, aux2_port, baudrate, "Collection")
    return CompleteRover(chassis, aux_motors)


if __name__ == '__main__':
    # Test the complete rover interface
    print("="*60)
    print("COMPLETE ROVER TEST (6 MOTORS)")
    print("="*60)
    
    rover = create_complete_rover(
        '/dev/ttyUSB0',  # FR
        '/dev/ttyUSB1',  # FL
        '/dev/ttyUSB2',  # BR
        '/dev/ttyUSB3',  # BL
        '/dev/ttyUSB4',  # AUX1
        '/dev/ttyUSB5'   # AUX2
    )
    
    try:
        if rover.connect_all():
            print("\n✓ All motors connected! Running test sequence...\n")
            
            print("Test 1: Drive forward for 2 seconds")
            rover.chassis.forward()
            time.sleep(2)
            
            print("Test 2: Stop drive")
            rover.chassis.stop()
            time.sleep(1)
            
            print("Test 3: Auxiliary motors forward for 2 seconds")
            rover.aux_motors.forward()
            time.sleep(2)
            
            print("Test 4: Auxiliary motors backward for 2 seconds")
            rover.aux_motors.backward()
            time.sleep(2)
            
            print("Test 5: Stop auxiliary")
            rover.aux_motors.stop()
            time.sleep(1)
            
            print("\n✓ Test complete!")
        else:
            print("\n✗ Not all motors connected - test aborted")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        rover.emergency_stop()
    
    finally:
        rover.disconnect_all()