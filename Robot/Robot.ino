#include <PID_v1.h>
#include <Buzzer.h>
#include <Arduino_LSM9DS1.h>
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"
#include "MotorController.h"
#include "Songs.h"
#include "Stopwatch.h"
#include "Gyro.h"
#include "PositionEstimator.h"

// TODO: refactor turnController into a class
double sensedAngle, angularSpeed, targetAngle;
PID turnController(&sensedAngle, &angularSpeed, &targetAngle, 0.2, 0, 0.05, DIRECT);
Buzzer buzzer(10);
LineSensor lineSensor(A3, A2);
Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);
MotorController leftMotorController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorController rightMotorController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);
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

  leftMotorController.reset();
  rightMotorController.reset();
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

  leftMotorController.reset();
  rightMotorController.reset();
  
  sensedAngle = gyro.getAngle();
  targetAngle = sensedAngle + degrees;

  Stopwatch setpointTimer;

  // Keep running until we've settled on an angle (so we don't stop
  // as soon as we get a single target measurement)
  while (setpointTimer.getElapsedTime() < SETTLING_TIME) {
    float dt = setpointTimer.lap();
    sensedAngle = gyro.getAngle();

    turnController.Compute();
    leftMotorController.setTargetVelocity(-angularSpeed);
    rightMotorController.setTargetVelocity(angularSpeed);

    leftMotorController.update();
    rightMotorController.update();

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
  rightMotorController.reset();

  while (true) {
    double kp = 0.25;
    double angularAdjustment = (gyro.getAngle() - startAngle) * kp;

    leftVelocityController.setTargetVelocity(BASE_SPEED + angularAdjustment);
    rightMotorController.setTargetVelocity(BASE_SPEED - angularAdjustment);

    leftVelocityController.update();
    rightMotorController.update();

    posEstimator.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }
}

/**
 * Drive straight a given number of inches
 */
void driveStraight(double inches) {
  Serial.println("Drive: " + String(inches) + "in.");

  leftMotorController.reset();
  rightMotorController.reset();

  leftMotorController.setTargetVelocity(BASE_SPEED);
  rightMotorController.setTargetVelocity(BASE_SPEED);

  
  while (!leftMotorController.reachedSetpoint() || !rightMotorController.reachedSetpoint()) {
    if (leftMotorController.getTargetPosition() >= inches) {
      leftMotorController.setControlMode(ControlMode::POSITION);
      leftMotorController.setTargetPosition(inches);
    }

    if (rightMotorController.getTargetPosition() >= inches) {
      rightMotorController.setControlMode(ControlMode::POSITION);
      rightMotorController.setTargetPosition(inches);
    }

    leftMotorController.update();
    rightMotorController.update();

    posEstimator.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotorController.setControlMode(ControlMode::VELOCITY);
  rightMotorController.setControlMode(ControlMode::VELOCITY);

  leftMotor.stop();
  rightMotor.stop();
}

/**
 * Centers the robot's center of rotation on a newly-sensed junction
 */
void centerOnJunction() {
  Serial.println("Centering on junction");
  driveStraight(ROBOT_HEIGHT_INCHES);
}

/**
 * Follow a line until a junction is reached.
 */
void driveUntilJunction() {
  Serial.println("Driving to Junction");

  leftMotorController.reset();
  rightMotorController.reset();
  
  while (lineSensor.identifyJunction() == Junction::LINE) {
    // Determine angular adjustment
    // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
    float skew = lineSensor.getSkew();
    leftMotorController.setTargetVelocity(BASE_SPEED - SKEW_ADJUSTMENT_FACTOR * skew);
    rightMotorController.setTargetVelocity(BASE_SPEED + SKEW_ADJUSTMENT_FACTOR * skew);

    leftMotorController.update();
    rightMotorController.update();

    posEstimator.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}
