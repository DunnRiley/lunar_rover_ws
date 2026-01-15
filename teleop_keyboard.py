"""
Skid-Steer Rover Teleoperation System with Auxiliary Motors
Now includes: 4 drive motors (USB0-3) + 2 auxiliary motors (USB4-5)
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
    AUX_1 = "aux_1"
    AUX_2 = "aux_2"


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
            self.serial.rts = False  # RTS LOW  ON
            self.serial.dtr = True   # DTR HIGH  OFF
            self._current_direction = MotorDirection.FORWARD
    
    def backward(self):
        """Move motor backward"""
        if self.serial:
            self.serial.rts = True   # RTS HIGH  OFF
            self.serial.dtr = False  # DTR LOW  ON
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


class AuxiliaryMotors:
    """
    Pair of auxiliary motors that move together
    """
    
    def __init__(self, motor1: Motor, motor2: Motor):
        """
        Initialize auxiliary motor pair
        
        Args:
            motor1: First auxiliary motor
            motor2: Second auxiliary motor
        """
        self.motor1 = motor1
        self.motor2 = motor2
        
    def connect(self):
        """Connect both motors"""
        self.motor1.connect()
        self.motor2.connect()
    
    def disconnect(self):
        """Disconnect both motors"""
        self.motor1.disconnect()
        self.motor2.disconnect()
    
    def stop(self):
        """Stop both motors"""
        self.motor1.stop()
        self.motor2.stop()
    
    def forward(self):
        """Move both motors forward"""
        self.motor1.forward()
        self.motor2.forward()
    
    def backward(self):
        """Move both motors backward"""
        self.motor1.backward()
        self.motor2.backward()


class SkidSteerRover:
    """
    Complete rover with 4 drive motors + 2 auxiliary motors
    """
    
    def __init__(self, fr_port: str, fl_port: str, br_port: str, bl_port: str, 
                 aux1_port: str, aux2_port: str, baudrate: int = 9600):
        """
        Initialize rover with six motors total
        
        Args:
            fr_port: Serial port for front-right motor (USB0)
            fl_port: Serial port for front-left motor (USB1)
            br_port: Serial port for back-right motor (USB2)
            bl_port: Serial port for back-left motor (USB3)
            aux1_port: Serial port for auxiliary motor 1 (USB4)
            aux2_port: Serial port for auxiliary motor 2 (USB5)
            baudrate: Communication baud rate
        """
        # Drive motors
        self.fr_motor = Motor(fr_port, MotorPosition.FRONT_RIGHT, baudrate)
        self.fl_motor = Motor(fl_port, MotorPosition.FRONT_LEFT, baudrate)
        self.br_motor = Motor(br_port, MotorPosition.BACK_RIGHT, baudrate)
        self.bl_motor = Motor(bl_port, MotorPosition.BACK_LEFT, baudrate)
        
        # Auxiliary motors
        aux1 = Motor(aux1_port, MotorPosition.AUX_1, baudrate)
        aux2 = Motor(aux2_port, MotorPosition.AUX_2, baudrate)
        self.aux_motors = AuxiliaryMotors(aux1, aux2)
        
        self._is_connected = False
    
    @property
    def drive_motors(self):
        """Return list of drive motors"""
        return [self.fr_motor, self.fl_motor, self.br_motor, self.bl_motor]
    
    @property
    def left_motors(self):
        """Return left side drive motors"""
        return [self.fl_motor, self.bl_motor]
    
    @property
    def right_motors(self):
        """Return right side drive motors"""
        return [self.fr_motor, self.br_motor]
    
    def connect(self):
        """Connect to all motors"""
        print("Connecting to rover motors...")
        print("Drive motors:")
        for motor in self.drive_motors:
            motor.connect()
        print("\nAuxiliary motors:")
        self.aux_motors.connect()
        self._is_connected = True
        print("\nRover fully connected and ready!")
    
    def disconnect(self):
        """Disconnect all motors"""
        print("Disconnecting rover motors...")
        for motor in self.drive_motors:
            motor.disconnect()
        self.aux_motors.disconnect()
        self._is_connected = False
        print("Rover disconnected")
    
    def stop(self):
        """Stop all drive motors"""
        for motor in self.drive_motors:
            motor.stop()
    
    def stop_all(self):
        """Stop ALL motors including auxiliary"""
        self.stop()
        self.aux_motors.stop()
    
    def forward(self):
        """Move rover forward - all drive motors forward"""
        for motor in self.drive_motors:
            motor.forward()
    
    def backward(self):
        """Move rover backward - all drive motors backward"""
        for motor in self.drive_motors:
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
    
    # Auxiliary motor controls
    def aux_forward(self):
        """Move auxiliary motors forward"""
        self.aux_motors.forward()
    
    def aux_backward(self):
        """Move auxiliary motors backward"""
        self.aux_motors.backward()
    
    def aux_stop(self):
        """Stop auxiliary motors"""
        self.aux_motors.stop()
    
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
        print("\n" + "="*60)
        print("ROVER TELEOP CONTROLS (6 MOTORS)")
        print("="*60)
        print("\nDRIVE CONTROLS:")
        print("  W - Forward")
        print("  S - Backward")
        print("  A - Turn Left")
        print("  D - Turn Right")
        print("  Z - Pivot Left")
        print("  C - Pivot Right")
        print("\n AUXILIARY MOTORS:")
        print("  Q - Auxiliary Forward")
        print("  E - Auxiliary Backward")
        print("\n  STOP:")
        print("  SPACE - Stop Drive Motors")
        print("  R - Stop Auxiliary Motors")
        print("  X - STOP ALL MOTORS")
        print("\n EXIT:")
        print("  ESC or Ctrl+C - Exit program")
        print("="*60 + "\n")
    
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
        # Drive controls
        if key == 'w':
            print(" Forward")
            self.rover.forward()
        elif key == 's':
            print(" Backward")
            self.rover.backward()
        elif key == 'a':
            print(" Turn Left")
            self.rover.turn_left()
        elif key == 'd':
            print(" Turn Right")
            self.rover.turn_right()
        elif key == 'z':
            print(" Pivot Left")
            self.rover.pivot_left()
        elif key == 'c':
            print(" Pivot Right")
            self.rover.pivot_right()
        
        # Auxiliary controls
        elif key == 'q':
            print("Auxiliary Forward")
            self.rover.aux_forward()
        elif key == 'e':
            print("Auxiliary Backward")
            self.rover.aux_backward()
        
        # Stop controls
        elif key == ' ':
            print("Stop Drive")
            self.rover.stop()
        elif key == 'r':
            print("Stop Auxiliary")
            self.rover.aux_stop()
        elif key == 'x':
            print("STOP ALL")
            self.rover.stop_all()
        
        # Exit
        elif key == '\x1b':  # ESC key
            print("  Exiting...")
            self.rover.stop_all()
            self._running = False


if __name__ == "__main__":
    # Configuration - 6 motors total
    print("="*60)
    print("6-MOTOR ROVER CONTROL SYSTEM")
    print("="*60)
    print("\nMotor Configuration:")
    print("  USB0  Front Right (FR) - Drive")
    print("  USB1  Front Left (FL) - Drive")
    print("  USB2  Back Right (BR) - Drive")
    print("  USB3  Back Left (BL) - Drive")
    print("  USB4  Auxiliary 1 - Collection/Excavation")
    print("  USB5  Auxiliary 2 - Collection/Excavation")
    
    FR_PORT = "/dev/ttyUSB0"
    FL_PORT = "/dev/ttyUSB1"
    BR_PORT = "/dev/ttyUSB2"
    BL_PORT = "/dev/ttyUSB3"
    AUX1_PORT = "/dev/ttyUSB4"
    AUX2_PORT = "/dev/ttyUSB5"

    # Create rover instance
    rover = SkidSteerRover(FR_PORT, FL_PORT, BR_PORT, BL_PORT, 
                          AUX1_PORT, AUX2_PORT)
    
    try:
        # Connect to rover
        rover.connect()
        
        # Run teleop
        teleop = KeyboardTeleop(rover)
        teleop.run()
    
    except KeyboardInterrupt:
        print("\n\n Interrupted by user")
    
    except Exception as e:
        print(f"\n Error occurred: {e}")
    
    finally:
        rover.stop_all()
        rover.disconnect()
        print("\n Shutdown complete. Goodbye!")