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

// Forward Declarations
void driveStraightForever(void);
void driveStraight(double inches);
void driveStraight2(double inches);
float getAngularSpeed();
void turnAngle(double degrees);
void turnAngle2(double degrees);
void driveUntilJunction();
void centerOnJunction();

void setup() {
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Songs::playStarWarsTheme(buzzer);

  lineSensor.begin();

  leftVelocityController.reset();
  rightVelocityController.reset();
}

void loop() {
  turnAngle(90);
  delay(2000);
  
  turnAngle(-90);
  delay(2000);
  
  turnAngle(180);
  delay(2000);
  
//  Junction junction = lineSensor.identifyJunction();
//  Serial.println(junctionAsString(junction));
//  Serial.println("Skew: " + String(lineSensor.getSkew()));
//  Serial.println("Skew2: " + String(lineSensor.getSkew2()));
//  
//  switch (junction) {
//    case Junction::DEAD_END:
//      turnAngle(180);
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
//      float skew = lineSensor.getSkew();
//      leftVelocityController.setTargetVelocity(BASE_SPEED + SKEW_ADJUSTMENT_FACTOR * skew);
//      rightVelocityController.setTargetVelocity(BASE_SPEED - SKEW_ADJUSTMENT_FACTOR * skew);
//
//      leftVelocityController.update();
//      rightVelocityController.update();
//      break;
//  }
}

void driveStraightForever() {
  Stopwatch angleStopwatch;
  double totalAngle = 0;

  while (true) {
    totalAngle += getAngularSpeed() * angleStopwatch.lap();

    double kp = 0.25;
    double angularAdjustment = -totalAngle * kp;

    leftVelocityController.setTargetVelocity(BASE_SPEED - angularAdjustment);
    rightVelocityController.setTargetVelocity(BASE_SPEED + angularAdjustment);

    leftVelocityController.update();
    rightVelocityController.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }
}

void driveStraight(double inches) {
  Serial.println("Drive: " + String(inches) + "in.");

  leftVelocityController.reset();
  rightVelocityController.reset();

  leftVelocityController.setTargetVelocity(BASE_SPEED);
  rightVelocityController.setTargetVelocity(BASE_SPEED);

  int baseTime = millis();
  int waitTime = (inches / BASE_SPEED) * 1000;
  while (millis() - baseTime < waitTime) {
    leftVelocityController.update();
    rightVelocityController.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}


void driveStraight2(double inches) {
  Serial.println("Drive2: " + String(inches) + "in.");

  leftPosController.reset();
  rightPosController.reset();

  leftPosController.setTargetPosition(inches);
  rightPosController.setTargetPosition(inches);

  while (!leftPosController.reachedSetpoint() || !rightPosController.reachedSetpoint()) {
    leftPosController.update();
    rightPosController.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}

/**
   Return angular speed in degrees / sec
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
   Turn the robot to a specified angle in degrees (positive = counter-clockwise)
*/
void turnAngle(double degrees) {
  Serial.println("Turning to angle " + String(degrees));
  rightMotor.stop();
  leftMotor.stop();

  Stopwatch setpointTimer;

  float sensedAngle = 0;
  float degreesPerSec = 40;

  // Keep running until we've settled on an angle or reached a timeout
  while (setpointTimer.getElapsedTime() < SETTLING_TIME || setpointTimer.getElapsedTime() > PID_TIMEOUT) {
    sensedAngle += getAngularSpeed() * setpointTimer.lap();

    // Use basic P control
    float targetAngle = (targetAngle >= degrees)
      ? degrees 
      : setpointTimer.getElapsedTime() * degreesPerSec;
    
    float angularSpeed = TURN_CONSTANTS.kp * (targetAngle - sensedAngle);

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

void centerOnJunction() {
  Serial.println("Centering on junction");
  driveStraight(ROBOT_HEIGHT_INCHES / 2);
  delay(2000);
}

void driveUntilJunction() {
  Serial.println("Driving to Junction");
  while (lineSensor.identifyJunction() == Junction::LINE) {
    // Determine angular adjustment
    // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
    float skew = lineSensor.getSkew();
    leftVelocityController.setTargetVelocity(BASE_SPEED - SKEW_ADJUSTMENT_FACTOR * skew);
    rightVelocityController.setTargetVelocity(BASE_SPEED + SKEW_ADJUSTMENT_FACTOR * skew);

    leftVelocityController.update();
    rightVelocityController.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }
}
