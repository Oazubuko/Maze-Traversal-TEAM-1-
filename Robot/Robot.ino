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
#include "Gyro.h"
#include "PositionEstimator.h"

// Turn Controller
double sensedAngle, angularSpeed, targetAngle;
PID turnController(&sensedAngle, &angularSpeed, &targetAngle, 0.2, 0, 0.05, DIRECT);

Buzzer buzzer(10);

LineSensor lineSensor(A3, A2);

Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);

MotorPositionController leftPosController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorPositionController rightPosController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);

MotorVelocityController leftVelocityController(leftMotor, leftPosController);
MotorVelocityController rightVelocityController(rightMotor, rightPosController);

Gyro gyro;

PositionEstimator posEstimator(leftMotor, rightMotor, gyro);

// Forward Declarations
void driveStraightForever(void);
void driveStraight(double inches);
void turnAngle(double degrees);
void driveUntilJunction();
void centerOnJunction();

void setup() {
  Serial.begin(9600);

  Songs::playStarWarsTheme(buzzer);

  gyro.begin();
  lineSensor.begin();
  
  turnController.SetMode(AUTOMATIC);
  turnController.SetOutputLimits(-MAX_TURN_SPEED, MAX_TURN_SPEED);
  turnController.SetSampleTime(PID_SAMPLE_PERIOD_MS);

  leftVelocityController.reset();
  rightVelocityController.reset();
}

void loop() {
  Junction junction = lineSensor.identifyJunction();
  Serial.println(junctionAsString(junction));

  switch (junction) {
    case Junction::DEAD_END:
    buzzer.sound(NOTE_E7, 200);
    turnAngle(180);
    break;      

    case Junction::T:
      buzzer.sound(NOTE_C7, 200);
    case Junction::RIGHT:
      buzzer.sound(NOTE_G7, 200);
      centerOnJunction();
      turnAngle(-90);
      break;

    case Junction::LEFT:
      buzzer.sound(NOTE_B7, 200);
      centerOnJunction();

      // If this is a + junction, and not a left junction, 
      // don't bother turning
      if (lineSensor.identifyJunction() == Junction::DEAD_END) {
        turnAngle(90);
      }
      
      break;

    case Junction::LINE:
    default:
      buzzer.sound(NOTE_A7, 200);
      driveUntilJunction();
      break;
  }
}

/**
 * Turn the robot to a specified angle in degrees (positive = counterclockwise)
 */
void turnAngle(double degrees) {
  Serial.println("Turning to " + String(degrees) + " degrees...");
  rightMotor.stop();
  leftMotor.stop();

  leftVelocityController.reset();
  rightVelocityController.reset();
  
  sensedAngle = gyro.getAngle();
  targetAngle = sensedAngle + degrees;

  Stopwatch setpointTimer;

  // Keep running until we've settled on an angle (so we don't stop
  // as soon as we get a single target measurement)
  while (setpointTimer.getElapsedTime() < SETTLING_TIME) {
    float dt = setpointTimer.lap();
    sensedAngle = gyro.getAngle();

    turnController.Compute();
    leftVelocityController.setTargetVelocity(-angularSpeed);
    rightVelocityController.setTargetVelocity(angularSpeed);

    leftVelocityController.update();
    rightVelocityController.update();

    posEstimator.update();

    if (fabs(sensedAngle - targetAngle) >= DEGREE_THRESHOLD) {
      setpointTimer.zeroOut();
    }

    Serial.println("Moving " + String(sensedAngle) + " -> " + String(targetAngle) + " @ " + String(angularSpeed));
    Serial.println("Time at Target: " + String(setpointTimer.getElapsedTime()));
    Serial.println("Angle Error (degrees): " + String(targetAngle - sensedAngle));

    delay(PID_SAMPLE_PERIOD_MS);
  }

  rightMotor.stop();
  leftMotor.stop();
}

void driveStraightForever() {
  Stopwatch angleStopwatch;
  float startAngle = gyro.getAngle();
  
  leftVelocityController.reset();
  rightVelocityController.reset();

  while (true) {
    double kp = 0.25;
    double angularAdjustment = (gyro.getAngle() - startAngle) * kp;

    leftVelocityController.setTargetVelocity(BASE_SPEED + angularAdjustment);
    rightVelocityController.setTargetVelocity(BASE_SPEED - angularAdjustment);

    leftVelocityController.update();
    rightVelocityController.update();

    posEstimator.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }
}

void driveStraight(double inches) {
  Serial.println("Drive: " + String(inches) + "in.");

  leftVelocityController.reset();
  rightVelocityController.reset();

  leftVelocityController.setTargetVelocity(BASE_SPEED);
  rightVelocityController.setTargetVelocity(BASE_SPEED);

  
  while (!leftPosController.reachedSetpoint() || !rightPosController.reachedSetpoint()) {
    if (leftPosController.getTargetPosition() <= inches) {
      leftVelocityController.update();
    } else {
      leftPosController.setTargetPosition(inches);
      leftPosController.update();
    }

    if (rightPosController.getTargetPosition() <= inches) {
      rightVelocityController.update();
    } else {
      rightPosController.setTargetPosition(inches);
      rightPosController.update();
    }

    posEstimator.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}

void centerOnJunction() {
  Serial.println("Centering on junction");
  driveStraight(ROBOT_HEIGHT_INCHES);
}

void driveUntilJunction() {
  Serial.println("Driving to Junction");

  leftVelocityController.reset();
  rightVelocityController.reset();
  
  while (lineSensor.identifyJunction() == Junction::LINE) {
    // Determine angular adjustment
    // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
    float skew = lineSensor.getSkew();
    leftVelocityController.setTargetVelocity(BASE_SPEED - SKEW_ADJUSTMENT_FACTOR * skew);
    rightVelocityController.setTargetVelocity(BASE_SPEED + SKEW_ADJUSTMENT_FACTOR * skew);

    leftVelocityController.update();
    rightVelocityController.update();

    posEstimator.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}
