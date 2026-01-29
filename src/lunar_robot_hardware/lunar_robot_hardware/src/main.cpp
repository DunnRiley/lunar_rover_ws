#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


// FL Wheel 0x01
// FR Wheel 0x02
// BL Wheel 0x03
// BR wheel 0x04
// ACTUATORS 0x08
// Servo1 0x10
//Servo2 0x11



// Speed Control 
// Signed Speed?
// 10000000 - correctlates to backwards 0
// 10000001 - correlates to backwards speed 2
// Say we do a signed 8 bit, 7 bits of speed
// so 128 positions of speed
// maybe if we receive a signed 1 thats backwards speed 2
// we receive an unsigned 4 thats speed 8


const uint8_t FrontRightPWM = 4;
const uint8_t BackRightPWM  = 5;
const uint8_t FrontLeftPWM = 7;
const uint8_t BackLeftPWM = 8;
const uint8_t LeftActuatorPWM = 9;
const uint8_t RightActuatorPWM = 6;
const uint8_t SERVO1PWM = 10;
const uint8_t SERVO2PWM = 11;
const uint8_t SERVO3PWM = 12;
const uint8_t SERVO4PWM = 44;

const uint8_t FrontRightDIR = 22;
const uint8_t BackRightDIR = 23;
const uint8_t FrontLeftDIR = 26;
const uint8_t BackLeftDIR = 27;
const uint8_t LeftActuatorDIR = 25;
const uint8_t RightActuatorDIR = 24;
const uint8_t ActuatorEncoder1A = 2;
const uint8_t ActuatorEncoder1B = 3;
const uint8_t ActuatorEncoder2A = 18;
const uint8_t ActuatorEncoder2B = 19;

//const uint8_t BackupActuatorEncoder1A = 38;
//const uint8_t BackupActuatorEncoder1B = 40;
//const uint8_t BackupActuatorEncoder2A = 39;
//const uint8_t BackupActuatorEncoder2B = 41;

const uint8_t FrontLeftEncoder_CS = 33;
const uint8_t FrontRightEncoder_CS = 28;
const uint8_t BackRightEncoder_CS = 29;
const uint8_t BackLeftEncoder_CS = 32;
//const uint8_t ExtraSPI_CS = 30;
//const uint8_t ExtraSPI_CS = 31;

const uint8_t Test = 13;

//GPIO Extras

// Extra
// GPIO	
// 1	VCC
// 2	GND
// 3	AD0
// 4	AD1
// 5	AD2
// 6	AD3
// 7	AD4
// 8	AD5
// 9	AD6
// 10	AD7
// 11	AD8
// 12	AD9
// 13	AD10
// 14	AD11
// 15	AD12
// 16	AD13
// 17	AD14
// 18	AD15

// Extra PWMS
// PWM
// D45
// D46

// put function declarations here:
int myFunction(int, int);

void setup() {

  // Setting the MEGA as a SPI master
  pinMode(53, OUTPUT);


  // Initializing Direction Control
  pinMode(FrontLeftDIR, OUTPUT);
  pinMode(FrontRightDIR, OUTPUT);
  pinMode(BackRightDIR, OUTPUT);
  pinMode(BackLeftDIR, OUTPUT);

  // Initializing PWM Outputs
  pinMode(FrontLeftPWM, OUTPUT);
  pinMode(FrontRightPWM, OUTPUT);
  pinMode(BackLeftPWM, OUTPUT);
  pinMode(BackRightPWM,OUTPUT);
  pinMode(LeftActuatorPWM, OUTPUT);
  pinMode(BackLeftPWM, OUTPUT);

  //Initialize Motor Encoders
  pinMode(FrontLeftEncoder_CS,OUTPUT);
  digitalWrite(FrontLeftEncoder_CS, HIGH);
  pinMode(BackLeftEncoder_CS, OUTPUT);
  digitalWrite(BackLeftEncoder_CS, HIGH);
  pinMode(FrontRightEncoder_CS, OUTPUT);
  digitalWrite(FrontRightEncoder_CS, HIGH);
  pinMode(BackRightEncoder_CS, OUTPUT);
  digitalWrite(BackRightEncoder_CS, HIGH);

  //Encoder Inputs
  pinMode(ActuatorEncoder1A, INPUT_PULLUP);
  pinMode(ActuatorEncoder1B, INPUT_PULLUP);
  pinMode(ActuatorEncoder2A, INPUT_PULLUP);
  pinMode(ActuatorEncoder2B, INPUT_PULLUP);

// Engage COMs
Serial.begin(115200);
// Setting a baud rate/transfer rate of 115200



}

void loop() {
  // put your main code here, to run repeatedly:


}

// put function definitions here:
int MotorDriving() {



}


int ActuatorMovement(){

}




// Current COMS idea
// [0xAA][Device][Signed_Speed][0x55]
// Intiially the arduino will wait for a start byte, begin recording data
// It will read the end byte to ensure that everything is received
// Unpackage the data and send to device