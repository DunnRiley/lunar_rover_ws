#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

// ── MPU6050 ───────────────────────────────────────────────────────────────
const uint8_t MPU_ADDR = 0x68;

int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
int16_t temp_raw;

float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;

float gx_bias = 0, gy_bias = 0, gz_bias = 0;

unsigned long lastIMUms = 0;
const unsigned long imuPeriodMs = 50;

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

const uint8_t FrontLeftEncoder_CS  = 33;
const uint8_t FrontRightEncoder_CS = 28;
const uint8_t BackRightEncoder_CS  = 29;
const uint8_t BackLeftEncoder_CS   = 32;

// ── Protocol ─────────────────────────────────────────────────────────────
const uint8_t START = 0xAA;
const uint8_t END   = 0x55;

uint8_t data[3];
uint8_t idx = 0;

enum RxState { WAIT_START, READ_DATA, WAIT_END };
RxState rxState = WAIT_START;

// ── Encoder state ────────────────────────────────────────────────────────
volatile long leftActuatorCount  = 0;
volatile long rightActuatorCount = 0;

// Send counts every 50 ms
unsigned long lastTelemetryMs = 0;
const unsigned long telemetryPeriodMs = 50;

// ── Function prototypes ──────────────────────────────────────────────────
void HandleInput(uint8_t device, uint8_t speed, uint8_t direction);
void MotorDriving(uint8_t pwmPin, uint8_t speed, uint8_t direction, uint8_t dirPin);
void DriveLeft(uint8_t speed, uint8_t direction);
void DriveRight(uint8_t speed, uint8_t direction);
void ActuatorMovement(uint8_t speed, uint8_t direction);
void STOPALL();
void Servomove(uint8_t speed);

bool initMPU();
bool readMPU();
void calibrateGyro();
void sendIMUTelemetry();
void writeInt16LE(HardwareSerial &port, int16_t v);

// Encoder helpers
void leftEncoderISR();
void rightEncoderISR();
void sendActuatorCounts();

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
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

  Serial.begin(115200);   // USB to PC: commands in + telemetry out

  // If using separate UART hardware, you can enable this too:
  // Serial2.begin(115200);

  attachInterrupt(digitalPinToInterrupt(ActuatorEncoder1A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ActuatorEncoder2A), rightEncoderISR, CHANGE);

  Wire.begin();

  if (initMPU()) {
    calibrateGyro();
  }
}

// ── Main loop ─────────────────────────────────────────────────────────────
void loop() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    switch (rxState) {
      case WAIT_START:
        if (b == START) {
          idx = 0;
          rxState = READ_DATA;
        }
        break;

      case READ_DATA:
        data[idx++] = b;
        if (idx >= 3) rxState = WAIT_END;
        break;

      case WAIT_END:
        if (b == END) {
          Serial.write(0xAA);   // ACK
          HandleInput(data[0], data[1], data[2]);
        }
        rxState = WAIT_START;
        break;
    }
  }

  if (millis() - lastTelemetryMs >= telemetryPeriodMs) {
    lastTelemetryMs = millis();
    sendActuatorCounts();
  }
}

// ── MPU6050 helpers ───────────────────────────────────────────────────────
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

  ax_g   = ax_raw / 16384.0f;
  ay_g   = ay_raw / 16384.0f;
  az_g   = az_raw / 16384.0f;

  gx_dps = (gx_raw - gx_bias) / 131.0f;
  gy_dps = (gy_raw - gy_bias) / 131.0f;
  gz_dps = (gz_raw - gz_bias) / 131.0f;

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

void sendIMUTelemetry() {
  Serial.write(START);
  Serial.write(0x10);

  writeInt16LE(Serial, ax_raw);
  writeInt16LE(Serial, ay_raw);
  writeInt16LE(Serial, az_raw);
  writeInt16LE(Serial, gx_raw);
  writeInt16LE(Serial, gy_raw);
  writeInt16LE(Serial, gz_raw);

  Serial.write(END);
}

void writeInt16LE(HardwareSerial &port, int16_t v) {
  port.write((uint8_t)(v & 0xFF));
  port.write((uint8_t)((v >> 8) & 0xFF));
}

// ── Encoder helpers ───────────────────────────────────────────────────────
// Uses channel A interrupt, reads channel B for direction
void leftEncoderISR() {
  bool a = digitalRead(ActuatorEncoder1A);
  bool b = digitalRead(ActuatorEncoder1B);

  if (a == b) leftActuatorCount++;
  else        leftActuatorCount--;
}

void rightEncoderISR() {
  bool a = digitalRead(ActuatorEncoder2A);
  bool b = digitalRead(ActuatorEncoder2B);

  if (a == b) rightActuatorCount++;
  else        rightActuatorCount--;
}

void sendActuatorCounts() {
  // avoid blocking if TX buffer is full
  if (Serial.availableForWrite() < 24) return;

  long l, r;
  noInterrupts();
  l = leftActuatorCount;
  r = rightActuatorCount;
  interrupts();

  Serial.print("ACT,");
  Serial.print(l);
  Serial.print(",");
  Serial.println(r);
}

// ── Motor helpers ─────────────────────────────────────────────────────────
void MotorDriving(uint8_t pwmPin, uint8_t speed, uint8_t direction, uint8_t dirPin) {
  analogWrite(pwmPin, speed);
  digitalWrite(dirPin, direction);
}

void DriveLeft(uint8_t speed, uint8_t direction) {
  analogWrite(FrontLeftPWM, speed);
  analogWrite(BackLeftPWM,  speed);
  digitalWrite(FrontLeftDIR, direction);
  digitalWrite(BackLeftDIR,  direction);
}

void DriveRight(uint8_t speed, uint8_t direction) {
  analogWrite(FrontRightPWM, speed);
  analogWrite(BackRightPWM,  speed);
  digitalWrite(FrontRightDIR, direction);
  digitalWrite(BackRightDIR,  direction);
}

void ActuatorMovement(uint8_t speed, uint8_t direction) {
  analogWrite(RightActuatorPWM, speed);
  analogWrite(LeftActuatorPWM,  speed);
  digitalWrite(RightActuatorDIR, direction);
  digitalWrite(LeftActuatorDIR,  direction);
}

void STOPALL() {
  analogWrite(FrontLeftPWM,    0);
  analogWrite(FrontRightPWM,   0);
  analogWrite(BackLeftPWM,     0);
  analogWrite(BackRightPWM,    0);
  analogWrite(LeftActuatorPWM, 0);
  analogWrite(RightActuatorPWM,0);
}

void Servomove(uint8_t speed) {
  myServo.write(speed);
}

void HandleInput(uint8_t device, uint8_t speed, uint8_t direction) {
  switch (device) {
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
    case 0xD4:
      MotorDriving(RightActuatorPWM, speed, direction, RightActuatorDIR);
      break;
    case 0xF7:
      MotorDriving(LeftActuatorPWM, speed, direction, LeftActuatorDIR);
      break;
    case 0x01:
      MotorDriving(FrontLeftPWM, speed, direction, FrontLeftDIR);
      break;
    case 0x02:
      MotorDriving(FrontRightPWM, speed, direction, FrontRightDIR);
      break;
    case 0x03:
      MotorDriving(BackLeftPWM, speed, direction, BackLeftDIR);
      break;
    case 0x04:
      MotorDriving(BackRightPWM, speed, direction, BackRightDIR);
      break;
    case 0xFF:
      STOPALL();
      break;
    default:
      break;
  }
}