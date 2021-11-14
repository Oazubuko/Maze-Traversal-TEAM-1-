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

PIDController turnController(TURN_CONSTANTS, -MAX_TURN_SPEED, MAX_TURN_SPEED, DEGREE_THRESHOLD);
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

  leftMotorController.reset();
  rightMotorController.reset();
}

void loop() {
  driveStraight(12);
  turnAngle(90);
//  Junction junction = lineSensor.identifyJunction();
//  Serial.println(junctionAsString(junction));
//
//  switch (junction) {
//    case Junction::DEAD_END:
//    buzzer.sound(NOTE_E7, 200);
//    turnAngle(180);
//    break;      
//
//    case Junction::T:
//      buzzer.sound(NOTE_C7, 200);
//    case Junction::RIGHT:
//      buzzer.sound(NOTE_G7, 200);
//      centerOnJunction();
//      turnAngle(-90);
//      break;
//
//    case Junction::LEFT:
//      buzzer.sound(NOTE_B7, 200);
//      centerOnJunction();
//
//      // If this is a + junction, and not a left junction, 
//      // don't bother turning
//      if (lineSensor.identifyJunction() == Junction::DEAD_END) {
//        turnAngle(90);
//      }
//      
//      break;
//
//    case Junction::LINE:
//    default:
//      buzzer.sound(NOTE_A7, 200);
//      driveUntilJunction();
//      break;
//  }
}


/**
 * Turn the robot to a specified angle in degrees (positive = counterclockwise)
 */
void turnAngle(double degrees) {
  Serial.println("Turning to " + String(degrees) + " degrees...");
  rightMotor.stop();
  leftMotor.stop();

  turnController.Reset();
  leftMotorController.reset();
  rightMotorController.reset();
  
  turnController.SetSetpoint(gyro.getAngle() + degrees);

  // Keep running until we've settled on an angle (so we don't stop
  // as soon as we get a single target measurement)
  while (!turnController.ReachedSetpoint()) {
    double motorSpinSpeed = turnController.Compute(gyro.getAngle());
    leftMotorController.setTargetVelocity(-motorSpinSpeed);
    rightMotorController.setTargetVelocity(motorSpinSpeed);

    leftMotorController.update();
    rightMotorController.update();

    posEstimator.update();

    turnController.Print();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  rightMotor.stop();
  leftMotor.stop();
}

/**
 * Drive straight a given number of inches
 */
void driveStraight(double inches) {
  Serial.println("Drive: " + String(inches) + "in.");

  leftMotorController.reset();
  rightMotorController.reset();

  leftMotorController.setMaxPosition(inches);
  rightMotorController.setMaxPosition(inches);
  
  float startAngle = gyro.getAngle();
  
  while (!leftMotorController.reachedSetpoint() || !rightMotorController.reachedSetpoint()) {
    double angularAdjustment = (gyro.getAngle() - startAngle) * ANGLE_ADJUSTMENT_FACTOR;
    
    leftMotorController.setTargetVelocity(BASE_SPEED + angularAdjustment);
    rightMotorController.setTargetVelocity(BASE_SPEED - angularAdjustment);

    leftMotorController.update();
    rightMotorController.update();

    posEstimator.update();

    leftMotorController.print();

    delay(PID_SAMPLE_PERIOD_MS);
  }

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
    float skew = lineSensor.getSkew2();
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
