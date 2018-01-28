/* 
Build the boy
*/
/* LIBRARIES */

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <PID_v1.h>

/* CONSTANTS */

#define US1trigPin 13
#define US1echoPin 12
#define servoPin   10
#define US2trigPin 9
#define US2echoPin 8
#define US2vccPin  7

#define right 0
#define left  1

#define servoAngleLeft  119.5
#define servoAngleRight 74.5


#define kp 10
#define kd 0.5
#define ki 0

#define defaultSpeed 100

#define arrayLength 5

/* GLOBALS */

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *dcMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *dcMotor2 = AFMS.getMotor(4);

Servo servo1;

double setPoint = 30;
double distance;
double controlSignal;
PID myPID(&distance, &controlSignal, &setPoint, kp, kd, ki, DIRECT);

int mode = left;

/* SETUP */

void setup() {  
  Serial.begin(9600);

  pinMode(US1trigPin, OUTPUT);
  pinMode(US1echoPin, INPUT);
  pinMode(US2trigPin, OUTPUT);
  pinMode(US2echoPin, INPUT);  
  pinMode(US2vccPin,  OUTPUT);

  digitalWrite(US2vccPin, HIGH);

  servo1.attach(servoPin);
  if (mode == left){
    servo1.write(servoAngleLeft);
  }
  else{
    servo1.write(servoAngleRight);
  }
  
  distance = getSideDistance();

  AFMS.begin();
  dcMotor1->setSpeed(0);
  dcMotor2->setSpeed(0); 
  dcMotor1->run(FORWARD);
  dcMotor2->run(FORWARD);

  myPID.SetOutputLimits(-100, 100);
  myPID.SetMode(AUTOMATIC);
}

/* LOOP */

void loop() {
  distance = getSideDistance();
  if (distance > 100){
    if (mode == left){
      dcMotor1->setSpeed(1.2 * 0.5 * defaultSpeed);
      dcMotor2->setSpeed(2 * defaultSpeed);
    }
    else{
      dcMotor1->setSpeed(1.2 * 2 * defaultSpeed);
      dcMotor2->setSpeed(0.5 * defaultSpeed);
    }
    while (distance > 100){
      distance = getSideDistance();
    }
  }
  if (getFrontDistance() < 50){
    if (mode == left){
      dcMotor1->setSpeed(0);
      dcMotor2->run(BACKWARD);
      dcMotor2->setSpeed(defaultSpeed);
    }
    else{
      dcMotor1->run(BACKWARD);
      dcMotor1->setSpeed(1.2 * defaultSpeed);
      dcMotor2->setSpeed(0);
    }
    while(getFrontDistance() < 100){
      
    }
    dcMotor1->run(FORWARD);
    dcMotor2->run(FORWARD);
  }
  
  myPID.Compute();

  if (mode == left){
    if (controlSignal > 0){
      dcMotor1->setSpeed(1.2 * (defaultSpeed + controlSignal));
      dcMotor2->setSpeed(defaultSpeed);
    }
    else{
      dcMotor1->setSpeed(1.2 * defaultSpeed);
      dcMotor2->setSpeed(defaultSpeed - controlSignal);
    }
  }
  else{
    if (controlSignal > 0){
      dcMotor1->setSpeed(1.2 * defaultSpeed);
      dcMotor2->setSpeed(defaultSpeed + controlSignal);
    }
    else{
      dcMotor1->setSpeed(1.2 * (defaultSpeed - controlSignal));
      dcMotor2->setSpeed(defaultSpeed);
    }
  }
  Serial.println(controlSignal);
}

/* FUNCTIONS */

double getSideDistance(){
  int trigPin;
  int echoPin;
  if (mode == left){
    trigPin = US2trigPin;
    echoPin = US2echoPin;
  }
  else{
    trigPin = US1trigPin;
    echoPin = US1echoPin;
  }
  
  return getDistance(trigPin, echoPin);
}

double getFrontDistance(){
  int trigPin;
  int echoPin;
  if (mode == left){
    trigPin = US1trigPin;
    echoPin = US1echoPin;
  }
  else{
    trigPin = US2trigPin;
    echoPin = US2echoPin;
  }
  
  return getDistance(trigPin, echoPin);
}

double getDistance(int trigPin, int echoPin){
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (duration/2) / 29.1;  
}

