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
#define servoAngleLeft  140
#define servoAngleRight 54

#define kp 1
#define kd 0
#define ki 0

#define defaultSpeed 40

#define arrayLength 5

/* GLOBALS */

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *dcMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *dcMotor2 = AFMS.getMotor(4);

Servo servo1;

double setPoint = 40;
double distance;
double controlSignal;
PID myPID(&distance, &controlSignal, &setPoint, kp, kd, ki, DIRECT);

double distances[arrayLength];

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
    initServoParallel(servoAngleRight, 180);
  }
  else{
    initServoParallel(0, servoAngleLeft);
  }

  distance = getDistance();
  for (int i = 0; i < arrayLength; i++){
    distances[i] = distance;
  }
  distance = getAverageDistance();

  AFMS.begin();
  dcMotor1->setSpeed(0);
  dcMotor2->setSpeed(0); 
  dcMotor1->run(FORWARD);
  dcMotor2->run(FORWARD);

  myPID.SetOutputLimits(-50, 50);
  myPID.SetMode(AUTOMATIC);
}

/* LOOP */

void loop() {
  adjustServo();
  distance = getDistance();
  addDistance(distance);
  distance = getAverageDistance();
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
}

/* FUNCTIONS */
void adjustServo(){
  double curAngle = servo1.read();
  double minDistance = getDistance();
  double minAngle = curAngle;

  double angles[2];
  if ((mode == left && controlSignal < 0) || (mode == right && controlSignal > 0)){
    angles[0] = curAngle - abs(controlSignal)/10;
    angles[1] = curAngle - abs(controlSignal)/5;
  }
  else{
    angles[0] = curAngle + abs(controlSignal)/10;
    angles[1] = curAngle + abs(controlSignal)/5;
  }

  double curDistance;
  for (int i = 0; i < 2; i++){
    servo1.write(angles[i]);
    delay(100);
    curDistance = getDistance();
    if (curDistance < minDistance){
      minDistance = curDistance;
      minAngle = angles[i];
    }
  }

  servo1.write(minAngle);
  delay(50);
}


void initServoParallel(int startAngle, int endAngle){
  for (int i = 0; i < 3; i++){
    getDistance();
    delay(30);
  }
  servo1.write(startAngle);
  delay(30);
  double minDistance = getDistance();
  double minAngle = startAngle;
  double curDistance;
  for (int i = startAngle; i <= endAngle; i+=5){
    servo1.write(i);
    delay(30);
    curDistance = getDistance();
    if (curDistance < minDistance){
      minDistance = curDistance;
      minAngle = i;
    }
  }
  servo1.write(minAngle);
}

double getDistance(){
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
  
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (duration/2) / 29.1;
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

