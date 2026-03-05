#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

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
const uint8_t ENC = 0xA5;

uint8_t data[3];
uint8_t idx = 0;
uint16_t currcount = 0;

enum RxState { WAIT_START, READ_DATA, WAIT_END };
RxState rxState = WAIT_START;

int16_t drivemin = 29325;
int16_t drivemax = 29375;

int16_t dumpmin = 32500;
int16_t dumpmax = 32000;

int16_t digmin = 28850;
int16_t digmax = 28875;

enum ActuatorState {CALIBRATE, DUMPBUCKET, DRIVEPOSITION, DIGPOSITION, STOP};
ActuatorState actuatorState = STOP;
uint8_t chksum = 0;

// ── Encoder state ────────────────────────────────────────────────────────
volatile uint16_t leftActuatorCount  = 32000;

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
void STOPACT();
void Servomove(uint8_t speed);


// Actuators
void drivepositionact();
void dumppositionact();
void digpositionact();
void calibrate();


//telemetry
void sendTelemetry();
void sendENCCount();
void sendIMUTelemetry();
void writeUInt16LE(HardwareSerial &port, int16_t val);
void writeInt32LE(HardwareSerial &port, int32_t val);



// MPU
bool initMPU();
bool readMPU();
void calibrateGyro();

// Encoder helpers
void leftEncoderISR();
// void rightEncoderISR();
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
  Serial2.begin(115200);

  attachInterrupt(digitalPinToInterrupt(ActuatorEncoder1A), leftEncoderISR, CHANGE);
 //  attachInterrupt(digitalPinToInterrupt(ActuatorEncoder2A), rightEncoderISR, CHANGE);

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
    switch (actuatorState) {
        case CALIBRATE:
            calibrate();
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
            STOPACT();
        break;
    }
  
  if (millis() - lastTelemetryMs >= telemetryPeriodMs) {
    lastTelemetryMs = millis();
    readMPU();
    sendTelemetry();
  }

}


void sendTelemetry(){
    chksum = 0;
    Serial2.write(START);
    sendIMUTelemetry();
    chksum ^= ENC;
    Serial2.write(ENC);
    sendENCCount();
    Serial2.write(chksum);
    Serial2.write(END);
}

void sendENCCount(){
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



//── MPU6050 helpers ───────────────────────────────────────────────────────
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

  ax_ms2   = ax_raw * 9.80665f/ 4096.0f;
  ay_ms2   = ay_raw * 9.80665f/ 4096.0f;
  az_ms2   = az_raw * 9.80665f/ 4096.0f;

  gx_dps = (gx_raw - gx_bias) / 131.0f;
  gy_dps = (gy_raw - gy_bias) / 131.0f;
  gz_dps = (gz_raw - gz_bias) / 131.0f;

  ax_mms2 = (int32_t)roundf(ax_ms2*1000.0f);
  ay_mms2 = (int32_t)roundf(ay_ms2*1000.0f);
  az_mms2 = (int32_t)roundf(az_ms2*1000.0f);

  gx_scale = (int32_t)roundf(gx_dps*1000.0f);
  gy_scale = (int32_t)roundf(gy_dps*1000.0f);
  gz_scale = (int32_t)roundf(gz_dps*1000.0f);

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

  // read encoder atomically
  uint16_t c;
  noInterrupts();
  c = leftActuatorCount;
  interrupts();

  unsigned long now = millis();

  if (!started) {
    started = true;
    lastCount = c;
    lastChangeMs = now;

    // start moving up (retract)
    analogWrite(RightActuatorPWM, 255);
    analogWrite(LeftActuatorPWM, 255);
    digitalWrite(RightActuatorDIR, 0);
    digitalWrite(LeftActuatorDIR, 0);
    return;
  }

  // detect motion
  if (c != lastCount) {
    lastCount = c;
    lastChangeMs = now;
  }

  // if no count change for 250ms, assume we've hit the end-stop (or stalled)
  if (now - lastChangeMs > 250) {
    analogWrite(RightActuatorPWM, 0);
    analogWrite(LeftActuatorPWM, 0);

    // set "zero" here:
    noInterrupts();
    leftActuatorCount = 32000;   // or 0, your choice
    interrupts();

    started = false; // calibration complete
  }
}






// Actuator Controls

void drivepositionact(){
    uint16_t leftCount;
    noInterrupts();
    leftCount = leftActuatorCount;
    interrupts();
    if (leftCount < drivemin - 50){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR,  0);
    }
    else if (leftCount < drivemin - 25){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM, 125);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR, 0);
    }
    else if (leftCount > drivemax + 50){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else if (leftCount > drivemax + 25){
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
    uint16_t leftCount;
    noInterrupts();
    leftCount = leftActuatorCount;
    interrupts();
    if (leftCount < dumpmin - 50){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR,  0);
    }
    else if (leftCount < dumpmin - 25){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM, 125);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR, 0);
    }
    else if (leftCount > dumpmax + 50){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else if (leftCount > dumpmax + 25){
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
    uint16_t leftCount;
    noInterrupts();
    leftCount = leftActuatorCount;
    interrupts();
    if (leftCount < digmin - 50){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR,  0);
    }
    else if (leftCount < digmin - 25){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM, 125);
        digitalWrite(RightActuatorDIR, 0);
        digitalWrite(LeftActuatorDIR, 0);
    }
    else if (leftCount > digmax + 50){
        analogWrite(RightActuatorPWM, 255);
        analogWrite(LeftActuatorPWM,  255);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else if (leftCount > digmax + 25){
        analogWrite(RightActuatorPWM, 125);
        analogWrite(LeftActuatorPWM,  125);
        digitalWrite(RightActuatorDIR, 1);
        digitalWrite(LeftActuatorDIR,  1);
    }
    else{
        actuatorState = STOP;
    }
}

// ACTUATOR ENCODER READOUTS

void leftEncoderISR() {
  bool a = digitalRead(ActuatorEncoder1A);
  bool b = digitalRead(ActuatorEncoder1B);

  if (a == b) leftActuatorCount++;
  else        leftActuatorCount--;
}

// void rightEncoderISR() {
//   bool a = digitalRead(ActuatorEncoder2A);
//   bool b = digitalRead(ActuatorEncoder2B);

//   if (a == b) 
//     rightActuatorCount++;
//   else        
//     rightActuatorCount--;
// }


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

void STOPACT(){
    analogWrite(LeftActuatorPWM, 0);
    analogWrite(RightActuatorPWM, 0);

}

void Servomove(uint8_t angle) {
    myServo.write(angle);
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
    // Sets the state machine for the actuator to calibrate
    // Will 0 out encoder counts to be 
    case 0xCA:
        actuatorState = CALIBRATE;
        break;
    // Sets the Encoder Counts to be recieved values
    case 0xCB:
        leftActuatorCount = (speed<<8) | direction;
        break;
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
// This Code takes in a 
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
    default:  
        break;
  }
}