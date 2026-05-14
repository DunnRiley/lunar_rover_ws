#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <avr/wdt.h>

// ── MPU6050 ───────────────────────────────────────────────────────────────
const uint8_t MPU_ADDR = 0x68;

int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
int16_t temp_raw;

float ax_ms2, ay_ms2, az_ms2;
int32_t ax_mms2, ay_mms2, az_mms2;
float gx_dps, gy_dps, gz_dps;
int32_t gx_scale, gy_scale, gz_scale;

float gx_bias = 0, gy_bias = 0, gz_bias = 0;

// Servo
Servo myServo;

// ── Pin definitions ──────────────────────────────────────────────────────
const uint8_t FrontRightPWM     = 4;
const uint8_t BackRightPWM      = 5;
const uint8_t FrontLeftPWM      = 7;
const uint8_t BackLeftPWM       = 8;
const uint8_t LeftActuatorPWM   = 9;
const uint8_t RightActuatorPWM  = 6;
const uint8_t SERVO1PWM         = 10;
const uint8_t SERVO2PWM         = 11;
const uint8_t SERVO3PWM         = 12;
const uint8_t SERVO4PWM         = 44;

const uint8_t FrontRightDIR     = 22;
const uint8_t BackRightDIR      = 23;
const uint8_t FrontLeftDIR      = 26;
const uint8_t BackLeftDIR       = 27;
const uint8_t LeftActuatorDIR   = 25;
const uint8_t RightActuatorDIR  = 24;

const uint8_t ActuatorEncoder1A = 2;   // Left actuator A
const uint8_t ActuatorEncoder1B = 3;   // Left actuator B
const uint8_t ActuatorEncoder2A = 18;  // Right actuator A
const uint8_t ActuatorEncoder2B = 19;  // Right actuator B

const uint8_t FrontLeftEncoder_CS  = 32;
const uint8_t FrontRightEncoder_CS = 28;
const uint8_t BackRightEncoder_CS  = 29;
const uint8_t BackLeftEncoder_CS   = 33;   // change if BL is actually wired elsewhere

// ── Protocol ─────────────────────────────────────────────────────────────
const uint8_t START = 0xAA;
const uint8_t END   = 0x55;
const uint8_t ENC   = 0xA5;
uint8_t data[5];
uint8_t idx = 0;

enum RxState { WAIT_START, READ_DATA, WAIT_END };
unsigned long rxLastByteMs = 0;
const unsigned long RX_TIMEOUT_MS = 30;
RxState rxState = WAIT_START;

int16_t drivemin = 30775;
int16_t drivemax = 30725;

int16_t digmax2 = 28750;
int16_t digmin2 = 28750;

int16_t dumpmax = 32500;
int16_t dumpmin = 32000;

int16_t digmin = 28950;
int16_t digmax = 28950;

enum ActuatorState {CALIBRATE, DUMPBUCKET, DRIVEPOSITION, DIGPOSITION,DIGPOSITION2, STOP};
ActuatorState actuatorState = STOP;
uint8_t chksum = 0;

// ── Encoder state ────────────────────────────────────────────────────────
volatile uint16_t leftActuatorCount  = 32000;

// Send counts every 500 ms
unsigned long lastTelemetryMs = 0;
const unsigned long telemetryPeriodMs = 500;

const uint8_t TELEMETRY_BYTES = 30;

// ═══════════════════════════════════════════════════════════════════════════
// ── BL-ONLY DISTANCE DRIVE SETTINGS ───────────────────────────────────────
// ═══════════════════════════════════════════════════════════════════════════
float WHEEL_DIAMETER_MM = 355.6f;
float GEAR_RATIO        = 168.0f;
float COUNTS_PER_MM     = 0.0f;

// 15-bit 0xDC format:
//   combined = (speed << 8) | direction
//   bit 15   = drive direction (0 = forward, 1 = reverse)
//   bits 14:0 = distance units
// With DIST_UNIT_MM = 1.0, raw units are millimetres.
const float DIST_UNIT_MM = 1.0f;
uint8_t DD_DRIVE_SPEED   = 120;

// Direction inversion for mirrored drivetrain.
bool invertLeftDriveDirection  = false;
bool invertRightDriveDirection = true;

// Only Back Left is used for distance measurement.
static uint16_t blPrevAngle = 0;
static int32_t  blPosition  = 0;

static uint16_t brPrevAngle = 0;
static int32_t  brPosition  = 0;

enum DriveDistState { DD_IDLE, DD_RUNNING};
static DriveDistState ddStatebl        = DD_IDLE;
static DriveDistState ddStatebr        = DD_IDLE;
static int32_t        ddStartBL      = 0;
static int32_t        ddStartBR      = 0;
static int32_t        ddTargetCountsbl = 0;
static int32_t        ddTargetCountsbr = 0;
static uint8_t        ddDirectionbl = 0;
static uint8_t        ddDirectionbr = 0;  // 0 = forward, 1 = reverse
uint8_t ddspeedbr = 0;
uint8_t ddspeedbl = 0;
bool ddblcont = false;
bool ddbrcont = false;
float kp = 0.05;
unsigned long         ddStartMsbl      = 0;
unsigned long         ddStartMsbr      = 0;

const unsigned long   DD_TIMEOUT_MS  = 30000;

enum EncodCommandState {IDLE, DUAL, DIFFERNTIALSTOP, CONTINUE};
static EncodCommandState autodrive = IDLE;
bool continuedrive = false;

// ── Function prototypes ──────────────────────────────────────────────────
void HandleInput(uint8_t device, uint8_t speed, uint8_t direction, uint8_t lobyte, uint8_t chksum);
void MotorDriving(uint8_t pwmPin, uint8_t speed, uint8_t direction, uint8_t dirPin);
void DriveLeft(uint8_t speed, uint8_t direction);
void DriveRight(uint8_t speed, uint8_t direction);
void ActuatorMovement(uint8_t speed, uint8_t direction);
void STOPALL();
void STOPACT();
void Servomove(uint8_t angle);

// Actuators
void drivepositionact();
void dumppositionact();
void digpositionact();
void digpositionact2();
void calibrate();

// Telemetry
void sendTelemetry();
void sendENCCount();
void sendIMUTelemetry();
void writeUInt16LE(HardwareSerial &port, uint16_t val);
void writeInt32LE(HardwareSerial &port, int32_t val);

// MPU
bool initMPU();
bool readMPU();
void calibrateGyro();

// Encoder helpers
void leftEncoderISR();

// BL wheel encoder helpers
uint8_t mapLeftDirection(uint8_t direction);
uint8_t mapRightDirection(uint8_t direction);
uint16_t readAS5048A(uint8_t csPin);
int16_t angleDelta14(uint16_t newAngle, uint16_t oldAngle);
void zeroBLEncoder();
void updateBLEncoder();
void updateDriveDistancebl();

void setup() {
  wdt_disable();
  pinMode(53, OUTPUT);

  pinMode(FrontLeftDIR,      OUTPUT);
  pinMode(FrontRightDIR,     OUTPUT);
  pinMode(BackRightDIR,      OUTPUT);
  pinMode(BackLeftDIR,       OUTPUT);
  pinMode(LeftActuatorDIR,   OUTPUT);
  pinMode(RightActuatorDIR,  OUTPUT);

  pinMode(FrontLeftPWM,     OUTPUT);
  pinMode(FrontRightPWM,    OUTPUT);
  pinMode(BackLeftPWM,      OUTPUT);
  pinMode(BackRightPWM,     OUTPUT);
  pinMode(LeftActuatorPWM,  OUTPUT);
  pinMode(RightActuatorPWM, OUTPUT);

  pinMode(FrontLeftEncoder_CS,  OUTPUT); digitalWrite(FrontLeftEncoder_CS,  HIGH);
  pinMode(BackLeftEncoder_CS,   OUTPUT); digitalWrite(BackLeftEncoder_CS,   HIGH);
  pinMode(FrontRightEncoder_CS, OUTPUT); digitalWrite(FrontRightEncoder_CS, HIGH);
  pinMode(BackRightEncoder_CS,  OUTPUT); digitalWrite(BackRightEncoder_CS,  HIGH);

  myServo.attach(SERVO1PWM);

  pinMode(ActuatorEncoder1A, INPUT_PULLUP);
  pinMode(ActuatorEncoder1B, INPUT_PULLUP);
  pinMode(ActuatorEncoder2A, INPUT_PULLUP);
  pinMode(ActuatorEncoder2B, INPUT_PULLUP);

  Serial.begin(115200);
  Serial2.begin(115200);

  attachInterrupt(digitalPinToInterrupt(ActuatorEncoder1A), leftEncoderISR, CHANGE);

  Wire.begin();
  Wire.setWireTimeout(2500, true);

  if (initMPU()) {
    calibrateGyro();
  }

  SPI.begin();
  COUNTS_PER_MM = (16384.0f * GEAR_RATIO) / (3.14159265f * WHEEL_DIAMETER_MM);
  zeroBLEncoder();
  wdt_enable(WDTO_250MS);
  wdt_reset();
}

void loop() {
  wdt_reset();
  while (Serial.available() > 0) {
    wdt_reset();
    uint8_t b = (uint8_t)Serial.read();
    rxLastByteMs = millis();

    switch (rxState) {
      case WAIT_START:
        if (b == START) {
          idx = 0;
          rxState = READ_DATA;
        }
        break;

      case READ_DATA:
        data[idx++] = b;
        if (idx >= 5) rxState = WAIT_END;
        break;

      case WAIT_END:
        if (b == END) {
          Serial.write(0xAA);
          HandleInput(data[0], data[1], data[2],data[3], data[4]);
          wdt_reset();
          rxState = WAIT_START;
        } else if (b == START) {
          idx = 0;
          rxState = READ_DATA;
        }
        break;
    }
  }

  if (rxState != WAIT_START && (millis() - rxLastByteMs) > RX_TIMEOUT_MS) {
    rxState = WAIT_START;
    idx = 0;
  }
  updateBLEncoder();
  updateBREncoder();
  switch (autodrive){
    case IDLE:
      break;
    case DUAL:
      dualdrive();
      break;
    case DIFFERNTIALSTOP:
      continuedrive = false;
      updateDriveDistancebr();
      updateDriveDistancebl();
      break;
    case CONTINUE:
      continuedrive = true;
      updateDriveDistancebr();
      updateDriveDistancebl();
      break;
    default:
      STOPALL();
  }

  switch (actuatorState) {
    case CALIBRATE:
      calibrate();
      break;
    case DRIVEPOSITION:
      drivepositionact();
      break;
    case DUMPBUCKET:
      dumppositionact();
      break;
    case DIGPOSITION:
      digpositionact();
      break;
    case DIGPOSITION2:
      digpositionact2();
      break;
    case STOP:
      break;
  }

  if (millis() - lastTelemetryMs >= telemetryPeriodMs) {
    if (Serial2.availableForWrite() >= TELEMETRY_BYTES) {
      if (readMPU()) {
        sendTelemetry();
        lastTelemetryMs += telemetryPeriodMs;
      }
    }
  }
  wdt_reset();
}

uint8_t mapLeftDirection(uint8_t direction) {
  return (invertLeftDriveDirection ? 1 : 0) ^ (direction ? 1 : 0);
}

uint8_t mapRightDirection(uint8_t direction) {
  return (invertRightDriveDirection ? 1 : 0) ^ (direction ? 1 : 0);
}

uint16_t readAS5048A(uint8_t csPin) {
  SPI.beginTransaction(SPISettings(100000UL, MSBFIRST, SPI_MODE1));

  digitalWrite(csPin, LOW);
  SPI.transfer16(0xFFFF);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(csPin, LOW);
  uint16_t raw = SPI.transfer16(0xC000);
  digitalWrite(csPin, HIGH);

  SPI.endTransaction();
  return raw & 0x3FFF;
}

int16_t angleDelta14(uint16_t newAngle, uint16_t oldAngle) {
  int16_t d = (int16_t)(newAngle - oldAngle);
  if (d > 8191)  d -= 16384;
  if (d < -8192) d += 16384;
  return d;
}

void zeroBLEncoder() {
  blPrevAngle = readAS5048A(BackLeftEncoder_CS);
  blPosition = 0;
}

void zeroBREncoder(){
  brPrevAngle = readAS5048A(BackRightEncoder_CS);
  brPosition = 0;
}

void updateBLEncoder() {
  uint16_t blAngle = readAS5048A(BackLeftEncoder_CS);
  blPosition += angleDelta14(blAngle, blPrevAngle);
  blPrevAngle = blAngle;
}

void updateBREncoder() {
  uint16_t brAngle = readAS5048A(BackRightEncoder_CS);
  brPosition += angleDelta14(brAngle, brPrevAngle);
  brPrevAngle = brAngle;
}

void dualdrive() {
  updateBREncoder();
  updateBLEncoder();
  if (ddStatebr != DD_RUNNING || ddStatebl != DD_RUNNING) return;

  int32_t travelBR = brPosition - ddStartBR;
  int32_t travelBL = blPosition - ddStartBL;

  if (travelBR < 0) travelBR = -travelBR;
  if (travelBL < 0) travelBL = -travelBL;

  int32_t error = travelBR - travelBL;
  int correction = (int)(kp * error);
  correction = constrain(correction, -10, 10);

  int rightPWM = ddspeedbr - correction;
  int leftPWM  = ddspeedbl + correction;

  rightPWM = constrain(rightPWM, 0, 255);
  leftPWM  = constrain(leftPWM, 0, 255);

  dualDriveLeft((uint8_t)leftPWM, ddDirectionbl);
  dualDriveRight((uint8_t)rightPWM, ddDirectionbr);

  if (travelBR >= ddTargetCountsbr || travelBL >= ddTargetCountsbl) {
    STOPALL();
  }
}

void updateDriveDistancebr(){
  if (ddStatebr != DD_RUNNING) return;
  int32_t travelBR = brPosition - ddStartBR;
  if (travelBR < 0) travelBR = -travelBR;
  if (travelBR >= ddTargetCountsbr) {
    ddbrcont = false;
    if (continuedrive && ddblcont){
      return;
    }
    else if(!ddbrcont && !ddblcont) {
      STOPALL();
      ddStatebr = DD_IDLE;
      return;
    }
    else if (!ddbrcont && !continuedrive){
      DriveRight(0,0);
      ddStatebr = DD_IDLE;
      return;
    }
    ddStatebr = DD_IDLE;
    STOPALL();
    return;
  }
}

void updateDriveDistancebl() {
  if (ddStatebl != DD_RUNNING) return;

  int32_t travelBL = blPosition - ddStartBL;
  if (travelBL < 0) travelBL = -travelBL;

  if (travelBL >= ddTargetCountsbl) {
    ddblcont = false;
    if (continuedrive && ddbrcont){
      return;
    }
    else if (!ddbrcont && !ddblcont){
      STOPALL();
      ddStatebl = DD_IDLE;
      return;
    }
    else if (!ddblcont && !continuedrive){
      DriveLeft(0,0);
      ddStatebl = DD_IDLE;
      return;
    }
    ddStatebl = DD_IDLE;
    STOPALL();
    return;
  }

  if (millis() - ddStartMsbr >= DD_TIMEOUT_MS || millis() - ddStartMsbl >= DD_TIMEOUT_MS ) {
    STOPALL();
  }
}

void sendTelemetry() {
  chksum = 0;
  Serial2.write(START);
  sendIMUTelemetry();
  chksum ^= ENC;
  Serial2.write(ENC);
  sendENCCount();
  Serial2.write(chksum);
  Serial2.write(END);
}

void sendENCCount() {
  uint16_t leftCount;
  noInterrupts();
  leftCount = leftActuatorCount;
  interrupts();
  writeUInt16LE(Serial2, leftCount);
}

void sendIMUTelemetry() {
  writeInt32LE(Serial2, ax_mms2);
  writeInt32LE(Serial2, ay_mms2);
  writeInt32LE(Serial2, az_mms2);
  writeInt32LE(Serial2, gx_scale);
  writeInt32LE(Serial2, gy_scale);
  writeInt32LE(Serial2, gz_scale);
}

void writeInt32LE(HardwareSerial &port, int32_t val) {
  uint8_t b0 = (uint8_t)(val & 0xFF);
  uint8_t b1 = (uint8_t)((val >> 8) & 0xFF);
  uint8_t b2 = (uint8_t)((val >> 16) & 0xFF);
  uint8_t b3 = (uint8_t)((val >> 24) & 0xFF);
  port.write(b0); chksum ^= b0;
  port.write(b1); chksum ^= b1;
  port.write(b2); chksum ^= b2;
  port.write(b3); chksum ^= b3;
}

void writeUInt16LE(HardwareSerial &port, uint16_t val) {
  uint8_t b0 = (uint8_t)(val & 0xFF);
  uint8_t b1 = (uint8_t)((val >> 8) & 0xFF);
  port.write(b0); chksum ^= b0;
  port.write(b1); chksum ^= b1;
}

bool initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  if (Wire.endTransmission(true) != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  if (Wire.endTransmission(true) != 0) return false;

  return true;
}

bool readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (n != 14) return false;

  ax_raw   = (Wire.read() << 8) | Wire.read();
  ay_raw   = (Wire.read() << 8) | Wire.read();
  az_raw   = (Wire.read() << 8) | Wire.read();
  temp_raw = (Wire.read() << 8) | Wire.read();
  gx_raw   = (Wire.read() << 8) | Wire.read();
  gy_raw   = (Wire.read() << 8) | Wire.read();
  gz_raw   = (Wire.read() << 8) | Wire.read();

  ax_ms2 = ax_raw * 9.80665f / 4096.0f;
  ay_ms2 = ay_raw * 9.80665f / 4096.0f;
  az_ms2 = az_raw * 9.80665f / 4096.0f;

  gx_dps = (gx_raw - gx_bias) / 131.0f;
  gy_dps = (gy_raw - gy_bias) / 131.0f;
  gz_dps = (gz_raw - gz_bias) / 131.0f;

  ax_mms2 = (int32_t)roundf(ax_ms2 * 1000.0f);
  ay_mms2 = (int32_t)roundf(ay_ms2 * 1000.0f);
  az_mms2 = (int32_t)roundf(az_ms2 * 1000.0f);

  gx_scale = (int32_t)roundf(gx_dps * 1000.0f);
  gy_scale = (int32_t)roundf(gy_dps * 1000.0f);
  gz_scale = (int32_t)roundf(gz_dps * 1000.0f);

  return true;
}

void calibrateGyro() {
  const int N = 500;
  long sx = 0, sy = 0, sz = 0;
  int count = 0;
  for (int i = 0; i < N; i++) {
    if (readMPU()) {
      sx += gx_raw;
      sy += gy_raw;
      sz += gz_raw;
      count++;
    }
    delay(5);
  }
  if (count > 0) {
    gx_bias = sx / (float)count;
    gy_bias = sy / (float)count;
    gz_bias = sz / (float)count;
  }
}

void calibrate() {
  static uint16_t lastCount = 0;
  static unsigned long lastChangeMs = 0;
  static bool started = false;

  uint16_t c;
  noInterrupts();
  c = leftActuatorCount;
  interrupts();

  unsigned long now = millis();

  if (!started) {
    started = true;
    lastCount = c;
    lastChangeMs = now;
    digitalWrite(RightActuatorDIR, 0);
    digitalWrite(LeftActuatorDIR, 0);
    analogWrite(RightActuatorPWM, 255);
    analogWrite(LeftActuatorPWM, 255);
    return;
  }

  if (c != lastCount) {
    lastCount = c;
    lastChangeMs = now;
  }

  if (now - lastChangeMs > 500) {
    analogWrite(RightActuatorPWM, 0);
    analogWrite(LeftActuatorPWM, 0);
    noInterrupts();
    leftActuatorCount = 32000;
    interrupts();
    actuatorState = STOP;
    started = false;
  }
}

void drivepositionact() {
  uint16_t leftCount;
  noInterrupts();
  leftCount = leftActuatorCount;
  interrupts();
  if      (leftCount < drivemin - 100) { digitalWrite(RightActuatorDIR, 0); digitalWrite(LeftActuatorDIR, 0); analogWrite(RightActuatorPWM, 255); analogWrite(LeftActuatorPWM, 255); }
  else if (leftCount < drivemin - 50)  { digitalWrite(RightActuatorDIR, 0); digitalWrite(LeftActuatorDIR, 0); analogWrite(RightActuatorPWM, 125); analogWrite(LeftActuatorPWM, 125); }
  else if (leftCount > drivemax + 100) { digitalWrite(RightActuatorDIR, 1); digitalWrite(LeftActuatorDIR, 1); analogWrite(RightActuatorPWM, 255); analogWrite(LeftActuatorPWM, 255); }
  else if (leftCount > drivemax + 50)  { digitalWrite(RightActuatorDIR, 1); digitalWrite(LeftActuatorDIR, 1); analogWrite(RightActuatorPWM, 125); analogWrite(LeftActuatorPWM, 125); }
  else { STOPACT(); actuatorState = STOP; }
}

void dumppositionact() {
  uint16_t leftCount;
  noInterrupts();
  leftCount = leftActuatorCount;
  interrupts();
  if      (leftCount < dumpmin - 100) { digitalWrite(RightActuatorDIR, 0); digitalWrite(LeftActuatorDIR, 0); analogWrite(RightActuatorPWM, 255); analogWrite(LeftActuatorPWM, 255); }
  else if (leftCount < dumpmin - 50)  { digitalWrite(RightActuatorDIR, 0); digitalWrite(LeftActuatorDIR, 0); analogWrite(RightActuatorPWM, 125); analogWrite(LeftActuatorPWM, 125); }
  else if (leftCount > dumpmax + 100) { digitalWrite(RightActuatorDIR, 1); digitalWrite(LeftActuatorDIR, 1); analogWrite(RightActuatorPWM, 255); analogWrite(LeftActuatorPWM, 255); }
  else if (leftCount > dumpmax + 50)  { digitalWrite(RightActuatorDIR, 1); digitalWrite(LeftActuatorDIR, 1); analogWrite(RightActuatorPWM, 125); analogWrite(LeftActuatorPWM, 125); }
  else { STOPACT(); actuatorState = STOP; }
}

void digpositionact() {
  uint16_t leftCount;
  noInterrupts();
  leftCount = leftActuatorCount;
  interrupts();
  if      (leftCount < digmin - 100) { digitalWrite(RightActuatorDIR, 0); digitalWrite(LeftActuatorDIR, 0); analogWrite(RightActuatorPWM, 255); analogWrite(LeftActuatorPWM, 255); }
  else if (leftCount < digmin - 50)  { digitalWrite(RightActuatorDIR, 0); digitalWrite(LeftActuatorDIR, 0); analogWrite(RightActuatorPWM, 125); analogWrite(LeftActuatorPWM, 125); }
  else if (leftCount > digmax + 100) { digitalWrite(RightActuatorDIR, 1); digitalWrite(LeftActuatorDIR, 1); analogWrite(RightActuatorPWM, 255); analogWrite(LeftActuatorPWM, 255); }
  else if (leftCount > digmax + 50)  { digitalWrite(RightActuatorDIR, 1); digitalWrite(LeftActuatorDIR, 1); analogWrite(RightActuatorPWM, 125); analogWrite(LeftActuatorPWM, 125); }
  else { STOPACT(); actuatorState = STOP; }
}


void digpositionact2() {
  uint16_t leftCount;
  noInterrupts();
  leftCount = leftActuatorCount;
  interrupts();
  if      (leftCount < digmin2 - 100) { digitalWrite(RightActuatorDIR, 0); digitalWrite(LeftActuatorDIR, 0); analogWrite(RightActuatorPWM, 255); analogWrite(LeftActuatorPWM, 255); }
  else if (leftCount < digmin2 - 50)  { digitalWrite(RightActuatorDIR, 0); digitalWrite(LeftActuatorDIR, 0); analogWrite(RightActuatorPWM, 125); analogWrite(LeftActuatorPWM, 125); }
  else if (leftCount > digmax2 + 100) { digitalWrite(RightActuatorDIR, 1); digitalWrite(LeftActuatorDIR, 1); analogWrite(RightActuatorPWM, 255); analogWrite(LeftActuatorPWM, 255); }
  else if (leftCount > digmax2 + 50)  { digitalWrite(RightActuatorDIR, 1); digitalWrite(LeftActuatorDIR, 1); analogWrite(RightActuatorPWM, 125); analogWrite(LeftActuatorPWM, 125); }
  else { STOPACT(); actuatorState = STOP; }
}


void leftEncoderISR() {
  bool a = digitalRead(ActuatorEncoder1A);
  bool b = digitalRead(ActuatorEncoder1B);
  if (a == b) leftActuatorCount++;
  else        leftActuatorCount--;
}

void MotorDriving(uint8_t pwmPin, uint8_t speed, uint8_t direction, uint8_t dirPin) {
  uint8_t dir = direction;

  if (dirPin == FrontLeftDIR || dirPin == BackLeftDIR) {
    dir = mapLeftDirection(direction);
  } else if (dirPin == FrontRightDIR || dirPin == BackRightDIR) {
    dir = mapRightDirection(direction);
  }

  analogWrite(pwmPin, 0);
  digitalWrite(dirPin, dir);
  delay(30);
  analogWrite(pwmPin, speed);
}

void DriveLeft(uint8_t speed, uint8_t direction) {
  uint8_t dir = mapLeftDirection(direction);

  analogWrite(FrontLeftPWM, 0);
  analogWrite(BackLeftPWM,  0);
  digitalWrite(FrontLeftDIR, dir);
  digitalWrite(BackLeftDIR,  dir);
  delay(15);
  analogWrite(FrontLeftPWM, speed);
  analogWrite(BackLeftPWM,  speed);
}

void DriveRight(uint8_t speed, uint8_t direction) {
  uint8_t dir = mapRightDirection(direction);

  analogWrite(FrontRightPWM, 0);
  analogWrite(BackRightPWM,  0);
  digitalWrite(FrontRightDIR, dir);
  digitalWrite(BackRightDIR,  dir);
  delay(15);
  analogWrite(FrontRightPWM, speed);
  analogWrite(BackRightPWM,  speed);
}
void dualDriveRight(uint8_t speed, uint8_t direction){
  digitalWrite(FrontRightDIR, direction);
  digitalWrite(BackRightDIR,  direction);
  delay(5);
  analogWrite(FrontRightPWM, speed);
  analogWrite(BackRightPWM,  speed);
}
void dualDriveLeft(uint8_t speed, uint8_t direction){
  digitalWrite(FrontLeftDIR, direction);
  digitalWrite(BackLeftDIR,  direction);
  delay(5);
  analogWrite(FrontLeftPWM, speed);
  analogWrite(BackLeftPWM,  speed);
}

void ActuatorMovement(uint8_t speed, uint8_t direction) {
  digitalWrite(RightActuatorDIR, direction);
  digitalWrite(LeftActuatorDIR,  direction);
  analogWrite(RightActuatorPWM, speed);
  analogWrite(LeftActuatorPWM,  speed);
}

void STOPALL() {
  analogWrite(FrontLeftPWM,     0);
  analogWrite(FrontRightPWM,    0);
  analogWrite(BackLeftPWM,      0);
  analogWrite(BackRightPWM,     0);
  analogWrite(LeftActuatorPWM,  0);
  analogWrite(RightActuatorPWM, 0);
  myServo.write(90);
  ddStatebl = DD_IDLE;
  ddStatebr = DD_IDLE;
}

void STOPACT() {
  analogWrite(LeftActuatorPWM,  0);
  analogWrite(RightActuatorPWM, 0);
}

void Servomove(uint8_t angle) {
  myServo.write(angle);
}

void HandleInput (uint8_t device, uint8_t speed, uint8_t direction, uint8_t lobyte, uint8_t chksum){
  uint8_t checky = 0^device^speed^direction^lobyte;
  if(checky!=chksum){
  return;
  }
  switch (device) {
    case 0xC8:{
    // Load the left Encoder Counts
      uint16_t combined = ((uint16_t)direction << 8) | lobyte;
      uint8_t packedDirection = (combined >> 15) & 0x01;   // 0=fwd, 1=rev
      uint16_t distanceUnits  = combined & 0x7FFF;         // 15-bit distance
      float dist_mm = (float)distanceUnits * DIST_UNIT_MM;
      int32_t targetCts = (int32_t)(dist_mm * COUNTS_PER_MM + 0.5f);
      ddTargetCountsbl = targetCts;
      ddDirectionbl    = packedDirection;
      ddspeedbl = speed;
      break;
  }

    case 0xC9:{
    // Load the Right Encoder Counts
      uint16_t combined = ((uint16_t)direction << 8) | lobyte;
      uint8_t packedDirection = (combined >> 15) & 0x01;   // 0=fwd, 1=rev
      uint16_t distanceUnits  = combined & 0x7FFF;         // 15-bit distance
      float dist_mm = (float)distanceUnits * DIST_UNIT_MM;
      int32_t targetCts = (int32_t)(dist_mm * COUNTS_PER_MM + 0.5f);
      ddTargetCountsbr = targetCts;
      ddDirectionbr    = packedDirection;
      ddspeedbr = speed;

      break;
  }
    // cont turn
    case  0xE7: {
      STOPALL();
      zeroBLEncoder();
      zeroBREncoder();
      ddStartBR = 0;
      ddStartBL = 0;
      ddbrcont = true;
      ddblcont = true;
      autodrive = CONTINUE;
      ddStartMsbr      = millis();
      ddStartMsbl      = millis();
      ddStatebl        = DD_RUNNING;
      ddStatebr        = DD_RUNNING;
      DriveLeft(ddspeedbl, ddDirectionbl);
      DriveRight(ddspeedbr, ddDirectionbr);
      updateBLEncoder();
      updateBREncoder();
      break;
  }

    // Isolated Turn
    case 0xE8: {
      STOPALL();
      zeroBLEncoder();
      zeroBREncoder();
      ddStartBR = 0;
      ddStartBL = 0;
      ddbrcont = true;
      ddblcont = true;
      autodrive = DIFFERNTIALSTOP;
      ddStartMsbr      = millis();
      ddStartMsbl      = millis();
      ddStatebl        = DD_RUNNING;
      ddStatebr        = DD_RUNNING;
      DriveLeft(ddspeedbl, ddDirectionbl);
      DriveRight(ddspeedbr, ddDirectionbr);
      updateBLEncoder();
      updateBREncoder();
      break;
    }


    case 0xDC: {
      uint16_t combined = ((uint16_t)direction << 8) | lobyte;
      uint8_t packedDirection = (combined >> 15) & 0x01;   // 0=fwd, 1=rev
      uint16_t distanceUnits  = combined & 0x7FFF;         // 15-bit distance
      float dist_mm = (float)distanceUnits * DIST_UNIT_MM;
      int32_t targetCts = (int32_t)(dist_mm * COUNTS_PER_MM + 0.5f);
      autodrive = DUAL;

      if (targetCts <= 0) break;

      STOPALL();
      delay(10);
      zeroBLEncoder();
      zeroBREncoder();

      ddStartBL      = 0;
      ddStartBR      = 0;
      ddTargetCountsbl = targetCts;
      ddTargetCountsbr = targetCts;
      ddDirectionbr    = packedDirection;
      ddDirectionbl    = !packedDirection;
      ddspeedbr = speed;
      ddspeedbl = speed;
      ddStartMsbr      = millis();
      ddStartMsbl      = millis();
      ddStatebl        = DD_RUNNING;
      ddStatebr        = DD_RUNNING;
      DriveLeft(ddspeedbl, ddDirectionbl);
      DriveRight(ddspeedbr, ddDirectionbr);
      updateBLEncoder();
      updateBREncoder();
      break;
    }

    case 0xA7:
      actuatorState = DIGPOSITION;
      break;
    case 0x93:
      actuatorState = DIGPOSITION2;
      break;
    case 0xA9:
      actuatorState = DRIVEPOSITION;
      break;
    case 0xB3:     
      actuatorState = DUMPBUCKET;
      break;
    case 0xCA:
      actuatorState = CALIBRATE;
      break;
    case 0xCB: {
      uint16_t v = ((uint16_t)speed << 8) | direction;
      noInterrupts();
      leftActuatorCount = v;
      interrupts();
      break;
    }
    case 0xB4:
      STOPALL();
      break;
    case 0x05:
      DriveLeft(speed, direction);
      break;
    case 0x06:
      DriveRight(speed, direction);
      break;
    case 0x08:
      ActuatorMovement(speed, direction);
      break;
    case 0x11:
      Servomove(speed);
      break;
    // case 0xD4:
    //   MotorDriving(RightActuatorPWM, speed, direction, RightActuatorDIR);
    //   break;
    // case 0xF7:
    //   MotorDriving(LeftActuatorPWM, speed, direction, LeftActuatorDIR);
    //   break;
    case 0x01:
      MotorDriving(FrontLeftPWM,  speed, direction, FrontLeftDIR);
      break;
    case 0x02:
      MotorDriving(FrontRightPWM, speed, direction, FrontRightDIR);
      break;
    case 0x03:
      MotorDriving(BackLeftPWM,   speed, direction, BackLeftDIR);
      break;
    case 0x04:
      MotorDriving(BackRightPWM,  speed, direction, BackRightDIR);
      break;
    case 0xFF:
      STOPALL();
      break;
    case 0xD1:
      readMPU();
      sendTelemetry();
      break;
    default:
      break;
  }
}
