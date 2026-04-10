import serial
import time
from dataclasses import dataclass

START_BYTE = 0xAA
END_BYTE = 0x55


@dataclass
class CommandDef:
    name: str
    device: int
    needs_speed: bool = True
    needs_direction: bool = True
    needs_lobyte: bool = True
    description: str = ""


COMMANDS = {
    "load_left": CommandDef(
        name="load_left",
        device=0xC8,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=True,
        description="Preload left side split-drive target using packed direction+distance high byte and low byte.",
    ),
    "load_right": CommandDef(
        name="load_right",
        device=0xC9,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=True,
        description="Preload right side split-drive target using packed direction+distance high byte and low byte.",
    ),
    "continue_turn": CommandDef(
        name="continue_turn",
        device=0xE7,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="Trigger split-drive continue mode.",
    ),
    "isolated_turn": CommandDef(
        name="isolated_turn",
        device=0xE8,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="Trigger split-drive differential-stop mode.",
    ),
    "distance_dual": CommandDef(
        name="distance_dual",
        device=0xDC,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=True,
        description="Start dual-drive straight distance mode using packed direction+distance.",
    ),
    "dig_position": CommandDef(
        name="dig_position",
        device=0xA7,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="Set actuator state to DIGPOSITION.",
    ),
    "drive_position": CommandDef(
        name="drive_position",
        device=0xA9,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="Set actuator state to DRIVEPOSITION.",
    ),
    "dump_bucket": CommandDef(
        name="dump_bucket",
        device=0xB3,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="Set actuator state to DUMPBUCKET.",
    ),
    "calibrate": CommandDef(
        name="calibrate",
        device=0xCA,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="Set actuator state to CALIBRATE.",
    ),
    "set_actuator_encoder": CommandDef(
        name="set_actuator_encoder",
        device=0xCB,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=False,
        description="Set leftActuatorCount = (speed << 8) | direction.",
    ),
    "stop_all_b4": CommandDef(
        name="stop_all_b4",
        device=0xB4,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="STOPALL via 0xB4.",
    ),
    "drive_left": CommandDef(
        name="drive_left",
        device=0x05,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=False,
        description="Drive left side.",
    ),
    "drive_right": CommandDef(
        name="drive_right",
        device=0x06,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=False,
        description="Drive right side.",
    ),
    "actuator_move": CommandDef(
        name="actuator_move",
        device=0x08,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=False,
        description="Move actuators.",
    ),
    "servo_move": CommandDef(
        name="servo_move",
        device=0x11,
        needs_speed=True,
        needs_direction=False,
        needs_lobyte=False,
        description="Servo move; speed byte is used as angle.",
    ),
    "front_left_motor": CommandDef(
        name="front_left_motor",
        device=0x01,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=False,
        description="Front left motor direct drive.",
    ),
    "front_right_motor": CommandDef(
        name="front_right_motor",
        device=0x02,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=False,
        description="Front right motor direct drive.",
    ),
    "back_left_motor": CommandDef(
        name="back_left_motor",
        device=0x03,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=False,
        description="Back left motor direct drive.",
    ),
    "back_right_motor": CommandDef(
        name="back_right_motor",
        device=0x04,
        needs_speed=True,
        needs_direction=True,
        needs_lobyte=False,
        description="Back right motor direct drive.",
    ),
    "stop_all_ff": CommandDef(
        name="stop_all_ff",
        device=0xFF,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="STOPALL via 0xFF.",
    ),
    "telemetry_once": CommandDef(
        name="telemetry_once",
        device=0xD1,
        needs_speed=False,
        needs_direction=False,
        needs_lobyte=False,
        description="Read MPU and send telemetry once.",
    ),
}


def xor_checksum(device: int, speed: int, direction: int, lobyte: int) -> int:
    return device ^ speed ^ direction ^ lobyte


def build_packet(device: int, speed: int = 0, direction: int = 0, lobyte: int = 0) -> bytes:
    chksum = xor_checksum(device, speed, direction, lobyte)
    return bytes([START_BYTE, device, speed, direction, lobyte, chksum, END_BYTE])


def pack_distance_command(distance_units: int, drive_direction: int) -> tuple[int, int]:
    """
    Pack a 15-bit distance and 1-bit direction into:
        direction_byte = high byte
        lobyte         = low byte

    Arduino reconstructs:
        combined = (direction << 8) | lobyte

    Bit 15 = drive direction
    Bits 14:0 = distance units
    """
    if not (0 <= distance_units <= 0x7FFF):
        raise ValueError("distance_units must be between 0 and 32767")
    if drive_direction not in (0, 1):
        raise ValueError("drive_direction must be 0 or 1")

    combined = (drive_direction << 15) | distance_units
    high_byte = (combined >> 8) & 0xFF
    low_byte = combined & 0xFF
    return high_byte, low_byte


def prompt_int(prompt: str, min_val: int, max_val: int, default: int | None = None) -> int:
    while True:
        raw = input(f"{prompt} ").strip()
        if raw == "" and default is not None:
            return default
        try:
            value = int(raw, 0)
            if min_val <= value <= max_val:
                return value
            print(f"Enter a value from {min_val} to {max_val}.")
        except ValueError:
            print("Enter a valid integer. Prefix with 0x for hex if you want.")


def choose_port() -> str:
    raw = input("Serial port [default COM6]: ").strip()
    return raw if raw else "COM6"


def choose_baud() -> int:
    raw = input("Baud [default 115200]: ").strip()
    return int(raw) if raw else 115200


def print_menu() -> None:
    print("\nAvailable commands:")
    for idx, key in enumerate(COMMANDS, start=1):
        cmd = COMMANDS[key]
        print(f"{idx:2d}. {key:20s}  device=0x{cmd.device:02X}  {cmd.description}")
    print(" 0. quit")


def get_command_key() -> str | None:
    keys = list(COMMANDS.keys())
    while True:
        print_menu()
        raw = input("\nChoose command by number or name: ").strip().lower()
        if raw in ("0", "q", "quit", "exit"):
            return None
        if raw in COMMANDS:
            return raw
        try:
            idx = int(raw)
            if 1 <= idx <= len(keys):
                return keys[idx - 1]
        except ValueError:
            pass
        print("Invalid selection.")


def get_command_bytes(cmd: CommandDef) -> tuple[int, int, int]:
    speed = 0
    direction = 0
    lobyte = 0

    if cmd.name in ("load_left", "load_right", "distance_dual"):
        print("\nThis command uses packed direction + 15-bit distance.")
        distance_units = prompt_int("Distance units (0..32767):", 0, 32767)
        drive_direction = prompt_int("Drive direction bit (0=fwd, 1=rev):", 0, 1)
        direction, lobyte = pack_distance_command(distance_units, drive_direction)
        speed = prompt_int("PWM speed (0..255):", 0, 255, default=120)
        return speed, direction, lobyte

    if cmd.name == "set_actuator_encoder":
        value = prompt_int("Encoder value to load (0..65535):", 0, 65535)
        speed = (value >> 8) & 0xFF
        direction = value & 0xFF
        return speed, direction, lobyte

    if cmd.name == "servo_move":
        speed = prompt_int("Servo angle byte (0..180 recommended):", 0, 255, default=90)
        return speed, direction, lobyte

    if cmd.needs_speed:
        speed = prompt_int("Speed/PWM (0..255):", 0, 255, default=120)
    if cmd.needs_direction:
        direction = prompt_int("Direction byte (0 or 1 usually):", 0, 255, default=0)
    if cmd.needs_lobyte:
        lobyte = prompt_int("Low byte (0..255):", 0, 255, default=0)

    return speed, direction, lobyte


def read_available(ser: serial.Serial, duration: float = 0.25) -> bytes:
    end_time = time.time() + duration
    data = bytearray()
    while time.time() < end_time:
        waiting = ser.in_waiting
        if waiting:
            data.extend(ser.read(waiting))
        time.sleep(0.01)
    return bytes(data)


def main() -> None:
    port = choose_port()
    baud = choose_baud()

    print(f"\nOpening {port} at {baud}...")
    with serial.Serial(port, baudrate=baud, timeout=0.1) as ser:
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("Connected.")
        while True:
            key = get_command_key()
            if key is None:
                print("Exiting.")
                break

            cmd = COMMANDS[key]
            print(f"\nSelected: {cmd.name} (0x{cmd.device:02X})")
            speed, direction, lobyte = get_command_bytes(cmd)

            pkt = build_packet(cmd.device, speed, direction, lobyte)

            print("\nSending packet:")
            print(" ".join(f"0x{b:02X}" for b in pkt))

            ser.write(pkt)
            ser.flush()

            reply = read_available(ser, duration=0.4)
            if reply:
                print("Reply bytes:")
                print(" ".join(f"0x{b:02X}" for b in reply))
            else:
                print("No immediate reply received.")

            print()


if __name__ == "__main__":
    main()
