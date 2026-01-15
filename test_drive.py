"""
Skid-Steer Rover Teleoperation System
Object-oriented design for controlling a 4-wheel rover via USB serial ports
"""

import serial
import time
from enum import Enum
from typing import Optional
import sys


class MotorPosition(Enum):
    """Enum for identifying motor positions"""
    FRONT_RIGHT = "front_right"
    FRONT_LEFT = "front_left"
    BACK_RIGHT = "back_right"
    BACK_LEFT = "back_left"


class MotorDirection(Enum):
    """Enum for motor direction states"""
    FORWARD = "forward"
    BACKWARD = "backward"
    STOP = "stop"


class Motor:
    """
    Represents a single motor controlled via serial port RTS/DTR pins
    """
    
    def __init__(self, port: str, position: MotorPosition, baudrate: int = 9600, timeout: int = 1):
        """
        Initialize motor controller
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            position: Motor position on rover
            baudrate: Communication baud rate
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.position = position
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._current_direction = MotorDirection.STOP
        
    def connect(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.stop()
            print(f"Connected to {self.position.value} motor on {self.port}")
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            raise
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.stop()
            self.serial.close()
            print(f"Disconnected from {self.position.value} motor")
    
    def stop(self):
        """Stop motor movement"""
        if self.serial:
            self.serial.rts = True
            self.serial.dtr = True
            self._current_direction = MotorDirection.STOP
    
    def forward(self):
        """Move motor forward"""
        if self.serial:
            self.serial.rts = False  # RTS LOW ON
            self.serial.dtr = True   # DTR HIGH OFF
            self._current_direction = MotorDirection.FORWARD
    
    def backward(self):
        """Move motor backward"""
        if self.serial:
            self.serial.rts = True   # RTS HIGH OFF
            self.serial.dtr = False  # DTR LOW ON
            self._current_direction = MotorDirection.BACKWARD
    
    def set_direction(self, direction: MotorDirection):
        """Set motor direction using enum"""
        if direction == MotorDirection.FORWARD:
            self.forward()
        elif direction == MotorDirection.BACKWARD:
            self.backward()
        else:
            self.stop()
    
    @property
    def current_direction(self) -> MotorDirection:
        """Get current motor direction"""
        return self._current_direction
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


class SkidSteerRover:
    """
    Skid-steer rover with 4 motors (front-right, front-left, back-right, back-left)
    """
    
    def __init__(self, fr_port: str, fl_port: str, br_port: str, bl_port: str, baudrate: int = 9600):
        """
        Initialize rover with four motors
        
        Args:
            fr_port: Serial port for front-right motor (USB0)
            fl_port: Serial port for front-left motor (USB1)
            br_port: Serial port for back-right motor (USB2)
            bl_port: Serial port for back-left motor (USB3)
            baudrate: Communication baud rate
        """
        self.fr_motor = Motor(fr_port, MotorPosition.FRONT_RIGHT, baudrate)
        self.fl_motor = Motor(fl_port, MotorPosition.FRONT_LEFT, baudrate)
        self.br_motor = Motor(br_port, MotorPosition.BACK_RIGHT, baudrate)
        self.bl_motor = Motor(bl_port, MotorPosition.BACK_LEFT, baudrate)
        self._is_connected = False
    
    @property
    def all_motors(self):
        """Return list of all motors"""
        return [self.fr_motor, self.fl_motor, self.br_motor, self.bl_motor]
    
    @property
    def left_motors(self):
        """Return left side motors"""
        return [self.fl_motor, self.bl_motor]
    
    @property
    def right_motors(self):
        """Return right side motors"""
        return [self.fr_motor, self.br_motor]
    
    def connect(self):
        """Connect to all motors"""
        print("Connecting to rover motors...")
        for motor in self.all_motors:
            motor.connect()
        self._is_connected = True
        print("Rover fully connected and ready!")
    
    def disconnect(self):
        """Disconnect all motors"""
        print("Disconnecting rover motors...")
        for motor in self.all_motors:
            motor.disconnect()
        self._is_connected = False
        print("Rover disconnected")
    
    def stop(self):
        """Stop all motors"""
        for motor in self.all_motors:
            motor.stop()
    
    def forward(self):
        """Move rover forward - all motors forward"""
        for motor in self.all_motors:
            motor.forward()
    
    def backward(self):
        """Move rover backward - all motors backward"""
        for motor in self.all_motors:
            motor.backward()
    
    def turn_left(self):
        """Turn left - left motors backward, right motors forward"""
        for motor in self.left_motors:
            motor.backward()
        for motor in self.right_motors:
            motor.forward()
    
    def turn_right(self):
        """Turn right - left motors forward, right motors backward"""
        for motor in self.left_motors:
            motor.forward()
        for motor in self.right_motors:
            motor.backward()
    
    def pivot_left(self):
        """Pivot left - left motors stop, right motors forward"""
        for motor in self.left_motors:
            motor.stop()
        for motor in self.right_motors:
            motor.forward()
    
    def pivot_right(self):
        """Pivot right - left motors forward, right motors stop"""
        for motor in self.left_motors:
            motor.forward()
        for motor in self.right_motors:
            motor.stop()
    
    def strafe_left(self):
        """Strafe left - diagonal movement (if mechanically possible)"""
        self.fl_motor.backward()
        self.fr_motor.forward()
        self.bl_motor.forward()
        self.br_motor.backward()
    
    def strafe_right(self):
        """Strafe right - diagonal movement (if mechanically possible)"""
        self.fl_motor.forward()
        self.fr_motor.backward()
        self.bl_motor.backward()
        self.br_motor.forward()
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


class KeyboardTeleop:
    """
    Keyboard-based teleoperation interface
    """
    
    def __init__(self, rover: SkidSteerRover):
        """
        Initialize teleop controller
        
        Args:
            rover: SkidSteerRover instance to control
        """
        self.rover = rover
        self._running = False
    
    def print_controls(self):
        """Print control instructions"""
        print("\n" + "="*50)
        print("ROVER TELEOP CONTROLS")
        print("="*50)
        print("W - Forward")
        print("S - Backward")
        print("A - Turn Left")
        print("D - Turn Right")
        print("Q - Pivot Left")
        print("E - Pivot Right")
        print("SPACE - Stop")
        print("X - Exit")
        print("="*50 + "\n")
    
    def run(self):
        """Run the teleoperation loop"""
        self.print_controls()
        self._running = True
        
        try:
            import tty
            import termios
            import select
            
            # Save terminal settings
            old_settings = termios.tcgetattr(sys.stdin)
            
            try:
                tty.setcbreak(sys.stdin.fileno())
                
                print("Ready for commands! (Press keys to control)")
                while self._running:
                    if select.select([sys.stdin], [], [], 0)[0]:
                        key = sys.stdin.read(1).lower()
                        self._handle_key(key)
                    time.sleep(0.01)
            
            finally:
                # Restore terminal settings
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        except ImportError:
            # Fallback for Windows or systems without termios
            print("Using simple input mode (press Enter after each command)")
            print("Ready for commands!")
            while self._running:
                key = input("Command: ").lower()
                if key:
                    self._handle_key(key[0])
    
    def _handle_key(self, key: str):
        """Handle keyboard input"""
        if key == 'w':
            print("Forward")
            self.rover.forward()
        elif key == 's':
            print("Backward")
            self.rover.backward()
        elif key == 'a':
            print("Turn Left")
            self.rover.turn_left()
        elif key == 'd':
            print("Turn Right")
            self.rover.turn_right()
        elif key == 'q':
            print("Pivot Left")
            self.rover.pivot_left()
        elif key == 'e':
            print("Pivot Right")
            self.rover.pivot_right()
        elif key == ' ':
            print("Stop")
            self.rover.stop()
        elif key == 'x':
            print("Exiting...")
            self.rover.stop()
            self._running = False

if __name__ == "__main__":
    # Configuration - 4 motors in order: FR, FL, BR, BL (USB0-3)
    print("="*50)
    print("4-WHEEL SKID-STEER ROVER CONTROL")
    print("="*50)
    print("\nMotor Configuration:")
    print("USB0 Front Right (FR)")
    print("USB1 Front Left (FL)")
    print("USB2 Back Right (BR)")
    print("USB3 Back Left (BL)")
    
    FR_PORT = "/dev/ttyUSB0"
    FL_PORT = "/dev/ttyUSB1"
    BR_PORT = "/dev/ttyUSB2"
    BL_PORT = "/dev/ttyUSB3"


    # Create rover instance
    rover = SkidSteerRover(FR_PORT, FL_PORT, BR_PORT, BL_PORT)
    
    try:
        # Connect to rover
        rover.connect()
        
        print("Keyboard Teleop")
        
        teleop = KeyboardTeleop(rover)
        teleop.run()

    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
    
    finally:
        rover.disconnect()
        print("\nShutdown complete. Goodbye!")