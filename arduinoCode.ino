#include <NewPing.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20
//Definiciones para NewPing para el uso del sonar
#define PIN_TRIGGER  A8
#define PIN_ECHO     A9
#define MAX_DISTANCE 400
//Definiciones de pines de lectura de sensores IR
#define IRPinL A0
#define IRPinC A1
#define IRPinR A2

MPU6050 mpu;
NewPing sonar(PIN_TRIGGER, PIN_ECHO, MAX_DISTANCE);

//Pines de Bluetooth
char dato = '\0';

//******************

/******************************************
            DROP THE PACKAGE
               SERVOMOTOR
 ******************************************/
#include <Servo.h>
Servo servo1;
//\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\//

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 173;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 60;
double Kd = 1.4 ;
double Ki = 70;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.3;
double motorSpeedFactorRight = 0.3;
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

//Variables de movimiento
int moveState = 0; //0 = balance; 1 = atras; 2 = adelante;
bool sequenceStatus = false;

//Variables de lectura de sensores IR - 0 Linea negra; 1 Color detectado
byte IRL, IRC, IRR;
//Variables de lectura de sensor ultrasonico
float distance;
const float blockageDistance = 10; //10 cm como rango de deteccion de bloqueo
bool blocked = false;
int blockWait = 3000;
unsigned long blockTime;


volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
/**
  turnLeft
  Mover a la izquierda
*/
void left() {
  motorController.turnLeft(MIN_ABS_SPEED, false);
}
/**
  turnLeft
  Mover a la izquierda
*/
void right() {
  motorController.turnRight(MIN_ABS_SPEED, false);
}
/**
  setForth
  Coloca la bandera para el movimiento hacia adelante
*/
void setForth()
{
  moveState = 2;
}
/**
  setBack
  Coloca la bandera para el movimiento hacia atras
*/
void setBack()
{
  moveState = 1;
}
/**
  setStop
  Coloca la bandera para el movimiento en equilibrio
*/
void setStop()
{
  moveState = 0;
}
/**
  Mueve hacia el robot hacia atras o hacia adelante según la bandera moveState
*/
void moveBackForth()
{
  if (moveState == 0)
    setpoint = originalSetpoint;
  else if (moveState == 1)
    setpoint = originalSetpoint - movingAngleOffset;
  else
    setpoint = originalSetpoint + movingAngleOffset;
}


void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  //Bluetooth***********
  pinMode(A10, OUTPUT); //salida del Led para probar
  Serial.begin(9600);
  //*********************

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  /******************************************
              DROP THE PACKAGE
                 SERVOMOTOR
   ******************************************/
  servo1.attach(11, 600, 1500);
  //\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\//

}


void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI + 180;
  }

  //Evaluacion de lecturas de sensores
  IRL = digitalRead(IRPinL);
  IRC = digitalRead(IRPinC);
  IRR = digitalRead(IRPinR);

  if ( IRL == 1 && IRC == 0 and IRR == 1) { //Condicion de inicio 101
    setForth();
    sequenceStatus = true;
  } else if ( IRL == 1 && IRC == 1 and IRR == 1 ) { //Condicion de final 000
    setStop();
    sequenceStatus = false;
  } else if ( IRL == 1 ) {
    left();
  } else if ( IRR == 1 ) {
    right();
  }

  distance = sonar.ping_cm();

  if ( !sequenceStatus && distance <= blockageDistance ) {
    setStop();
    blocked = true;
    blockTime = millis();
  } else if ( sequenceStatus && millis() - blockTime > blockWait ) {
    blocked = false;
    setForth();
  }

  //BLuetooth**********
  if (Serial.available()) {
    LecturaDato();
  }
  //******************
}

//********Bluetoth*****
void LecturaDato() {
  int tmp = 0;
  dato = Serial.read();

  /* manual */
  if (dato == '0') {

  }
  /* automatic */
  else if (dato == '1') {

  }
  /* front */
  else if (dato == '2') {
    setForth();
  }
  /* back */
  else if (dato == '3') {
    setBack();
  }
  /* left */
  else if (dato == '4') {
    left();
  }
  /* right */
  else if (dato == '5') {
    right();
  }
  /* drop the package */
  else if (dato == '9') {
    dropPackage();
  }

}
/*
   Method to drop the package
*/
void dropPackage()
{
  servo1.write(0);
  delay(100);
  servo1.write(30);
  delay(100);
  servo1.write(0);
  delay(100);
  servo1.write(30);
  delay(100);
  servo1.write(0);
  delay(100);
  servo1.write(84);
  delay(150);
}
