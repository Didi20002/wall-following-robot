/* 
Build the boy
*/

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <PID_v1.h>

#define US1trigPin 13
#define US1echoPin 12
#define servo_Pin   10
#define US2trigPin 9
#define US2echoPin 8
#define US2vccPin  7

#define servoAngleLeft  140
#define servoAngleRight 54

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *dcMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *dcMotor2 = AFMS.getMotor(4);

Servo servo1;

#define right 0
#define left  1

#define kp 1.5
#define kd 0
#define ki 0
double setPoint = 30;
double distance;
double controlSignal;
PID myPID(&distance, &controlSignal, &setPoint, kp, kd, ki, DIRECT);

#define arrayLength 5
double distances[arrayLength];

#define defaultSpeed 40

int mode = left;

void setup() {  

  //Serial.begin(9600);
  
  servo1.attach(servo_Pin);
  if (mode = left){
    servo1.write(servoAngleLeft);
  }
  else{
    servo1.write(servoAngleRight);
  }

  pinMode(US1trigPin, OUTPUT);
  pinMode(US1echoPin, INPUT);
  pinMode(US2trigPin, OUTPUT);
  pinMode(US2echoPin, INPUT);  
  pinMode(US2vccPin,  OUTPUT);

  digitalWrite(US2vccPin, HIGH);

  AFMS.begin();
  setSpeed(0);  
  dcMotor1->run(FORWARD);
  dcMotor2->run(FORWARD);

  distance = getDistance(US1trigPin, US1echoPin);
  for (int i = 0; i < arrayLength; i++){
    distances[i] = distance;
  }
  distance = getAverageDistance();
  myPID.SetOutputLimits(-50, 50);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  if (mode == right){
    distance = getDistance(US1trigPin, US1echoPin);
  }
  else{
    distance = getDistance(US2trigPin, US2echoPin);
  }
  addDistance(distance);
  distance = getAverageDistance();
  myPID.Compute();
  //Serial.println(controlSignal);

  if (mode == left){
    if (controlSignal > 0){
      dcMotor1->setSpeed(1.5 * (defaultSpeed + controlSignal));
      dcMotor2->setSpeed(defaultSpeed);
    }
    else{
      dcMotor1->setSpeed(1.5 * defaultSpeed);
      dcMotor2->setSpeed(defaultSpeed - controlSignal);
    }
  }
  else{
    if (controlSignal > 0){
      dcMotor1->setSpeed(1.5 * defaultSpeed);
      dcMotor2->setSpeed(defaultSpeed + controlSignal);
    }
    else{
      dcMotor1->setSpeed(1.5 * (defaultSpeed - 1.5 * -controlSignal));
      dcMotor2->setSpeed(defaultSpeed);
    }
  }
  delay(100);
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

void setSpeed(int speed){
  dcMotor1->setSpeed(speed);
  dcMotor2->setSpeed(speed);
}

void addDistance(double newDistance){
  for (int i = 0; i < arrayLength - 1; i++){
    distances[i] = distances[i + 1];
  }
  distances[arrayLength - 1] = newDistance;
}

double getAverageDistance(){
  double sum = 0;
  for (int i = 0; i < arrayLength; i++){
    sum += distances[i];
  }
  return sum/arrayLength;
}

