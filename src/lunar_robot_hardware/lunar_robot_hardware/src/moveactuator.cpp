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

int16_t drivemin = -50;
int16_t drivemax = 50;

int16_t dumpmin = 2600;
int16_t dumpmax = 2700;

int16_t digmin = -450;
int16_t digmax = -400;

enum ActuatorState {DUMPBUCKET, DRIVEPOSITION, DIGPOSITION, STOP};
ActuatorState actuatorState = STOP;

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
void drivepositionact();
void dumppositionact();
void digpositionact();

// bool initMPU();
// bool readMPU();
// void calibrateGyro();
// void sendIMUTelemetry();
// void writeInt16LE(HardwareSerial &port, int16_t v);

// Encoder helpers
void leftEncoderISR();
void rightEncoderISR();
// void sendActuatorCounts();

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

//   if (initMPU()) {
//     calibrateGyro();
//   }
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
    switch (actuatorState) {
        case DRIVEPOSITION:
            drivepositionact();
        break;

        case DUMPBUCKET:
            dumppositionact();
        break;

        case DIGPOSITION:
            digpositionact();
        break;

        case STOP:
            STOPALL();
        break;
    }
  }

//   if (millis() - lastTelemetryMs >= telemetryPeriodMs) {
//     lastTelemetryMs = millis();
//     sendActuatorCounts();
//   }
void drivepositionact(){
    long leftCount;
    noInterrupts();
    leftCount = leftActuatorCount;
    interrupts();
    if (leftCount < drivemin - 100){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR,  0);
    }
    else if (leftCount < drivemin - 50){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM, 125);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR, 0);
    }
    else if (leftCount > drivemax + 100){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else if (leftCount > drivemax + 50){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM,  125);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else{
        actuatorState = STOP;
    }
}

void dumppositionact(){
    long leftCount;
    noInterrupts();
    leftCount = leftActuatorCount;
    interrupts();
    if (leftCount < dumpmin - 100){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR,  0);
    }
    else if (leftCount < dumpmin - 50){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM, 125);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR, 0);
    }
    else if (leftCount > dumpmax + 100){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else if (leftCount > dumpmax + 50){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM,  125);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else{
        actuatorState = STOP;
    }
}

void digpositionact(){
    long leftCount;
    noInterrupts();
    leftCount = leftActuatorCount;
    interrupts();
    if (leftCount < digmin - 100){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR,  0);
    }
    else if (leftCount < digmin - 50){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM, 125);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR, 0);
    }
    else if (leftCount > digmax + 100){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else if (leftCount > digmax + 50){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM,  125);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else{
        actuatorState = STOP;
    }
}



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


void STOPALL(){
    actuatorState = STOP;
    analogWrite(RightActuatorPWM, 0);
    analogWrite(LeftActuatorPWM,  0);
}


void HandleInput(uint8_t device, uint8_t speed, uint8_t direction) {
    switch (device) {
    case 0xA7:
      actuatorState = DIGPOSITION;
      break;
    case 0xA9:
      actuatorState = DRIVEPOSITION;
      break;
    case 0xB3:
      actuatorState = DUMPBUCKET;
      break;
    case 0xB4:
      STOPALL();
      break;
  }
}