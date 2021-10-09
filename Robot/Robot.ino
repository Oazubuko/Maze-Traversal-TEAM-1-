#include <math.h>
#include <Arduino_LSM9DS1.h>
#include <PID_v1.h>
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"
#include "MotorPositionController.h"
#include "MotorVelocityController.h"

Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);

MotorPositionController leftPosController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorPositionController rightPosController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);

MotorVelocityController leftVelocityController(leftMotor, leftPosController);
MotorVelocityController rightVelocityController(rightMotor, rightPosController);

void setup() {
  Serial.begin(9600);
  
  leftVelocityController.setTargetVelocity(5);
  rightVelocityController.setTargetVelocity(5);
}

void loop() {
  leftVelocityController.update();
  rightVelocityController.update();
  delay(10);
}
