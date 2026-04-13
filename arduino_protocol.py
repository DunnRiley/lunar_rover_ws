"""
arduino_protocol.py
===================
Single source of truth for the 7-byte serial protocol.

PACKET:  [0xAA][device][speed][direction][lobyte][checksum][0x55]
checksum = device ^ speed ^ direction ^ lobyte   (XOR, no leading 0 needed)

DISTANCE ENCODING (0xDC, 0xC8, 0xC9):
  combined    = (direction << 8) | lobyte    (16 bits)
  bit 15      = 0=forward, 1=reverse
  bits 14..0  = distance in mm (max 32767 mm = 32.7 m)

0xDC STRAIGHT DRIVE:
  Arduino inverts packedDir for left side: ddDirectionbl = !packedDirection
  After mapLeft/mapRight this means:
    packedDir=0  -> FORWARD
    packedDir=1  -> REVERSE

0xC8 / 0xC9 TURN LOAD:
  No inversion — packedDir stored directly.
  mapLeft(d) = d,  mapRight(d) = 1^d  (because invertRight=True)
  Sending SAME packedDir to C8 and C9 produces OPPOSITE physical directions -> pivot.
    packedDir=0 -> CW pivot  (left back, right fwd)
    packedDir=1 -> CCW pivot (left fwd, right back)
  NOTE: verify on your physical robot, swap if backward.

IMPORTANT — Arduino does NOT print "DIST_DONE".
dualdrive() silently calls STOPALL() when done.
The sequencer must use time-based or telemetry-based completion detection.
"""

START = 0xAA
END   = 0x55

# Commands
DIST_DRIVE     = 0xDC
LOAD_LEFT      = 0xC8
LOAD_RIGHT     = 0xC9
TURN_ISOLATED  = 0xE8   # each side stops when it hits its own target
TURN_CONTINUE  = 0xE7   # slower side waits for faster, then both stop
TURN_START_NEW = 0xDD   # newer firmware turn-start command

DRIVE_LEFT     = 0x05
DRIVE_RIGHT    = 0x06
ACT_BOTH       = 0x08
SERVO          = 0x11
STOP_FF        = 0xFF
STOP_B4        = 0xB4

ACT_DIG        = 0xA7
ACT_DRIVE_POS  = 0xA9
ACT_DUMP       = 0xB3
ACT_CAL        = 0xCA
REQ_TELEM      = 0xD1

SERVO_STOP = 90
SERVO_CW   = 135
SERVO_CCW  = 45

MAX_DRIVE_SPEED = 190   # Arduino firmware cap for encoder commands


def pkt(device: int, speed: int = 0, direction: int = 0, lobyte: int = 0) -> bytes:
    """Build a correctly checksummed 7-byte packet."""
    d, sp, di, lo = device & 0xFF, speed & 0xFF, direction & 0xFF, lobyte & 0xFF
    chk = d ^ sp ^ di ^ lo
    return bytes([START, d, sp, di, lo, chk, END])


def _encode_dist(mm: float, reverse: bool):
    """Return (direction_byte, lobyte) for the distance encoding."""
    units    = int(min(0x7FFF, max(0, round(abs(mm)))))
    combined = ((1 if reverse else 0) << 15) | units
    return (combined >> 8) & 0xFF, combined & 0xFF


def straight(metres: float, speed: int = 120) -> bytes:
    """0xDC straight drive. Positive = forward, negative = reverse."""
    speed = min(speed, MAX_DRIVE_SPEED)
    db, lo = _encode_dist(abs(metres) * 1000.0, metres < 0)
    return pkt(DIST_DRIVE, speed, db, lo)


def load_left_wheel(mm: float, speed: int, reverse: bool = False) -> bytes:
    """0xC8: load left wheel target."""
    db, lo = _encode_dist(mm, reverse)
    return pkt(LOAD_LEFT, min(speed, MAX_DRIVE_SPEED), db, lo)


def load_right_wheel(mm: float, speed: int, reverse: bool = False) -> bytes:
    """0xC9: load right wheel target."""
    db, lo = _encode_dist(mm, reverse)
    return pkt(LOAD_RIGHT, min(speed, MAX_DRIVE_SPEED), db, lo)


def start_isolated() -> bytes:
    """0xE8: start — each side stops independently."""
    return pkt(TURN_ISOLATED)


def start_continue() -> bytes:
    """0xE7: start — both sides wait until both complete."""
    return pkt(TURN_CONTINUE)

def start_turn_new() -> bytes:
    """0xDD: start turn (newer firmware)."""
    return pkt(TURN_START_NEW)


def pivot_packets(arc_mm: float, speed: int, clockwise: bool, start_cmd: int = TURN_START_NEW) -> list:
    """
    3-packet sequence for a pivot turn.
    arc_mm  = how far each wheel travels (track_width/2 * angle_radians)
    clockwise = True -> CW, False -> CCW

    Direction encoding:
      CW  (clockwise):     packedDir=0 for both sides
      CCW (counter-CW):    packedDir=1 for both sides
    """
    speed   = min(speed, MAX_DRIVE_SPEED)
    reverse = not clockwise   # CW=packedDir=0=not reversed, CCW=packedDir=1=reversed
    db, lo  = _encode_dist(arc_mm, reverse)
    return [
        pkt(LOAD_LEFT,     speed, db, lo),
        pkt(LOAD_RIGHT,    speed, db, lo),
        pkt(start_cmd),
    ]


def stop() -> bytes:
    return pkt(STOP_FF)


def actuator_preset(target: str) -> bytes:
    """target = 'dig', 'drive', or 'dump'"""
    d = {'dig': ACT_DIG, 'drive': ACT_DRIVE_POS, 'dump': ACT_DUMP}.get(target.lower())
    if d is None:
        raise ValueError(f"Bad actuator target '{target}'")
    return pkt(d)


def actuator_manual(speed: int, extend: bool) -> bytes:
    return pkt(ACT_BOTH, min(255, speed), 0 if extend else 1)


def servo_angle(angle: int) -> bytes:
    return pkt(SERVO, max(0, min(180, angle)))
