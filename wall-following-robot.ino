/* 
Build the boy
*/

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

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

void setup() {
  servo1.attach(servo_Pin);
  servo1.write(servoAngleLeft);

  pinMode(US1trigPin, OUTPUT);
  pinMode(US1echoPin, INPUT);
  pinMode(US2trigPin, OUTPUT);
  pinMode(US2echoPin, INPUT);  
  pinMode(US2vccPin,  OUTPUT);

  digitalWrite(US2vccPin, HIGH);

  AFMS.begin();
  dcMotor1->run(FORWARD);
  dcMotor2->run(FORWARD);

  delay(2000);
}

void loop() {  
  int distance = getDistance(US1trigPin, US1echoPin);

  if (distance < 10){
    setSpeed(0);
  }
  else{
    setSpeed(100);
  }
  delay(100);
}

int getDistance(int trigPin, int echoPin){
  long duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (int) ((duration/2) / 29.1);
}

void setSpeed(int speed){
  dcMotor1->setSpeed(speed);
  dcMotor2->setSpeed(speed);
}

