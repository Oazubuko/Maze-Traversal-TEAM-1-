#include <PID_v1.h>
#include <Buzzer.h>
#include <Arduino_LSM9DS1.h>
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"
#include "MotorPositionController.h"
#include "MotorVelocityController.h"
#include "Songs.h"
#include "Stopwatch.h"

Buzzer buzzer(10);
LineSensor lineSensor(A3, A2);

Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);

MotorPositionController leftPosController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorPositionController rightPosController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);

MotorVelocityController leftVelocityController(leftMotor, leftPosController);
MotorVelocityController rightVelocityController(rightMotor, rightPosController);

void driveStraightForever() {
  Stopwatch angleStopwatch;
  double totalAngle = 0;
  
  while (true) {
    totalAngle += getAngularSpeed() * angleStopwatch.lap() ;
  
    double kp = 0.25;
    double angularAdjustment = -totalAngle * kp;    
  
    leftVelocityController.setTargetVelocity(BASE_SPEED - angularAdjustment);
    rightVelocityController.setTargetVelocity(BASE_SPEED + angularAdjustment);  
    
    leftVelocityController.update();
    rightVelocityController.update();

    delay(PID_SAMPLE_TIME_MS);
  }  
}

void driveStraight(double inches) {
  leftVelocityController.reset();
  rightVelocityController.reset();

  leftVelocityController.setTargetVelocity(BASE_SPEED);
  rightVelocityController.setTargetVelocity(BASE_SPEED);

  int baseTime = millis();
  int waitTime = (inches / BASE_SPEED) * 1000;
  while (millis() - baseTime < waitTime) {
    leftVelocityController.update();
    rightVelocityController.update();

    leftPosController.print();
    rightPosController.print();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}


void driveStraight2(double inches) {
  leftPosController.reset();
  rightPosController.reset();

  leftPosController.setTargetPosition(inches);
  rightPosController.setTargetPosition(inches);
  
  while (!leftPosController.reachedSetpoint() || !rightPosController.reachedSetpoint()) {
    leftPosController.update();
    rightPosController.update();

    leftPosController.print();
    rightPosController.print();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}

/**
 * Return angular speed in degrees / sec
 */
float getAngularSpeed() {
  if (!IMU.gyroscopeAvailable()) {
    return 0;
  }

  float rotationX, rotationY, rotationZ;
  IMU.readGyroscope(rotationX, rotationY, rotationZ);

  return rotationZ - GYRO_BIAS;
}

/**
 * Turn the robot to a specified angle in degrees (positive = clockwise)
 */
void turnAngle(float degrees) {
  rightMotor.stop();
  leftMotor.stop();

  Stopwatch setpointTimer;

  double sensedAngle = 0;
  double totalError = 0;

  // Keep running until we've settled on an angle (so we don't stop
  // as soon as we get a single target measurement)
  while (setpointTimer.getElapsedTime() < SETTLING_TIME) {
    sensedAngle += getAngularSpeed() * setpointTimer.lap();

    // Use basic P control
    double angularSpeed = TURN_CONSTANTS.kp * (degrees - sensedAngle);

    leftVelocityController.setTargetVelocity(-angularSpeed);
    rightVelocityController.setTargetVelocity(angularSpeed);

    leftVelocityController.update();
    rightVelocityController.update();

    if (fabs(degrees - sensedAngle) > DEGREE_THRESHOLD) {
      setpointTimer.zeroOut();
    }

    Serial.println("Moving " + String(sensedAngle) + " -> " + String(degrees));
    Serial.println("Time at Target: " + String(setpointTimer.getElapsedTime()));

    delay(PID_SAMPLE_PERIOD_MS);
  }

  rightMotor.stop();
  leftMotor.stop();
}

void turnAngle2(double degrees) {
  leftVelocityController.reset();
  rightVelocityController.reset();

  double forwardSpeed = (degrees > 0) ? BASE_SPEED : -BASE_SPEED;
  leftVelocityController.setTargetVelocity(-forwardSpeed);
  rightVelocityController.setTargetVelocity(forwardSpeed);

  int baseTimeMs = millis();
  double radians = degrees / 180 * PI;
  int timeToDriveMs = ROBOT_RADIUS_INCHES * radians / forwardSpeed * 1000;

  Serial.println("Driving for " + String(timeToDriveMs) + " ms");
  
  while (millis() - baseTimeMs < timeToDriveMs) {
    leftVelocityController.update();
    rightVelocityController.update();

    leftPosController.print();
    rightPosController.print();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}

void centerOnJunction() {
  driveStraight(ROBOT_HEIGHT_INCHES / 2);
}

void setup() {
  Serial.begin(9600);
  IMU.begin();
  Songs::playStarWarsTheme(buzzer);

  leftVelocityController.reset();
  rightVelocityController.reset();
}

void loop() {
  delay(PID_SAMPLE_PERIOD_MS);
  
//  turnAngle2(-90);
//  delay(1000);
//  turnAngle2(90);
//  delay(1000);

//  float skew = lineSensor.getSkew();
//  Junction junction = lineSensor.identifyJunction();

//  switch (junction) {
//    case Junction::DEAD_END:
//      leftMotor.stop();
//      rightMotor.stop();
//      break;
//
//    case Junction::T:
//    case Junction::RIGHT:
//      centerOnJunction();
//      turnAngle(-90);
//      break;
//
//    case Junction::LEFT:
//      centerOnJunction();
//      turnAngle(90);
//      break;
//
//    case Junction::LINE:
//    default:
//      // Determine angular adjustment
//      // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
//      leftVelocityController.setTargetVelocity(BASE_SPEED - MAX_SPEED_ADJUSTMENT * skew);
//      rightVelocityController.setTargetVelocity(BASE_SPEED + MAX_SPEED_ADJUSTMENT * skew);
//  }


//  // Only print ocassionally
//  if (millis() % 1000 <= 1) {
//    Serial.println("Skew1 = " + String(skew));
//    Serial.println("Skew2 = " + String(lineSensor.getSkew2()));
//    Serial.println("Driving w/ speeds:\t" + String(leftVelocityController.getTargetVelocity()) + "\t" + String(rightVelocityController.getTargetVelocity()));
//    Serial.println("Junction Type: " + junctionAsString(junction));
//    Serial.println();
//  }
}
