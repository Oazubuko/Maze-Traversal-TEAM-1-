#include <math.h>
#include <Arduino_LSM9DS1.h>
#include <PID_v1.h>
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"
#include "MotorPositionController.h"

// Turn Controller
double sensedAngle, angularSpeed, targetAngle;
PID turnController(&sensedAngle, &angularSpeed, &targetAngle, 0.025, 0.03, 0.00125, DIRECT);

Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);

MotorPositionController leftPosController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorPositionController rightPosController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);

LineSensor lineSensor(A3, A2);

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
 * Turn the robot to a specified angle in degrees (positive = clockwise)
 */
void turnAngle(float degrees) {
  rightMotor.stop();
  leftMotor.stop();
  
  sensedAngle = 0;
  targetAngle = degrees;
  float dt = 50;
  int prevMicros = micros();
  float timeAtTarget = 0;

  // Keep running until we've settled on an angle (so we don't stop
  // as soon as we get a single target measurement)
  while (timeAtTarget < SETTLING_TIME) {
    int nowMicros = micros();
    dt = (nowMicros - prevMicros) * 1e-6;
    prevMicros = nowMicros;

    sensedAngle += getAngularSpeed() * dt;
    turnController.Compute();

    rightMotor.driveAtSpeed(angularSpeed);
    leftMotor.driveAtSpeed(-angularSpeed);

    if (fabs(targetAngle - sensedAngle) <= DEGREE_THRESHOLD) {
      timeAtTarget += dt;
    } else {
      timeAtTarget = 0;
    }

    Serial.print("Moving ");
    Serial.print(sensedAngle);
    Serial.print(" -> ");
    Serial.println(targetAngle);

    Serial.print("Time at Target: ");
    Serial.println(timeAtTarget);

    Serial.print("Angle Error (degrees): ");
    Serial.println(targetAngle - sensedAngle);

    delay(20);
  }

  rightMotor.stop();
  leftMotor.stop();
}

/**
   Drives the robot for the specified distance in centimeters
*/
void driveInches(float distanceInInches) {
  leftMotor.stop();
  rightMotor.stop();
  
  // Zero out the encoder values
  leftMotor.resetSensors();
  rightMotor.resetSensors();

  float leftError = distanceInInches;
  float rightError = distanceInInches;
  float leftErrorIntegral = 0;
  float rightErrorIntegral = 0;

  float angularError = 0;

  float dt = 50;
  int prevMicros = micros();

  while (fabs(leftError) > DISTANCE_THRESHOLD_INCHES || fabs(rightError) > DISTANCE_THRESHOLD_INCHES) {
    int nowMicros = micros();
    dt = (nowMicros - prevMicros) * 1e-6;
    prevMicros = nowMicros;

    angularError += getAngularSpeed() * dt;
    float angleAdjustment = angularError * turningConstants.kp;

    leftError = distanceInInches - rightMotor.getInchesDriven();
    rightError = distanceInInches - rightMotor.getInchesDriven();
    rightErrorIntegral += rightError * dt;
    leftErrorIntegral += leftError * dt;

    float leftSpeed = leftError * distanceConstants.kp
                      + leftErrorIntegral * distanceConstants.ki
                      + angleAdjustment;

    float rightSpeed = rightError * distanceConstants.kp
                       + rightErrorIntegral * distanceConstants.ki
                       - angleAdjustment;

    leftMotor.driveAtSpeed(leftSpeed);
    rightMotor.driveAtSpeed(rightSpeed);

    Serial.print("Left Error (in): ");
    Serial.print(leftError);
    Serial.print("\tIntegral: ");
    Serial.print(leftErrorIntegral);
    Serial.print("\tSpeed: ");
    Serial.println(leftSpeed);

    Serial.print("Right Error (in): ");
    Serial.print(rightError);
    Serial.print("\tIntegral: ");
    Serial.print(rightErrorIntegral);
    Serial.print("\tSpeed: ");
    Serial.println(rightSpeed);

    Serial.print("Angular Error: ");
    Serial.println(rightError);

    Serial.println();

    delay(20);
  }

  leftMotor.stop();
  rightMotor.stop();
}

void inchForward() {
  leftMotor.driveAtSpeed(0.1);
  rightMotor.driveAtSpeed(0.1);
  delay(1);
  leftMotor.stop();
  rightMotor.stop();
}

void setup() {
  Serial.begin(9600);

  turnController.SetMode(AUTOMATIC);
  turnController.SetOutputLimits(-0.2, 0.2);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  lineSensor.begin();
  leftMotor.begin();
  rightMotor.begin();
}

void loop() {
  rightMotor.stop();
  rightMotor.resetSensors();
  rightPosController.setTargetPosition(10);
//  leftMotor.stop();
//  leftMotor.resetSensors();
//  leftPosController.setTargetPosition(10);
  
  while (!rightPosController.reachedSetpoint()) {
//    leftPosController.update();
    rightPosController.update();
    delay(10);
  }

//  leftMotor.stop();
  rightMotor.stop();
  rightPosController.adjustPIDConstants();
  delay(5000);
  
//  constexpr float baseSpeed = 0.135;
//  constexpr float maxSpeedAdjustment = 0.08;
//  float leftSpeed, rightSpeed;
//
//  float skew = lineSensor.getSkew();
//
//  Junction junction = lineSensor.identifyJunction();
//
//  switch (junction) {
//    case Junction::DEAD_END:
//      leftMotor.driveAtSpeed(-baseSpeed * 1.25);
//      rightMotor.driveAtSpeed(-baseSpeed * 1.25);
//      break;
//
//    case Junction::T:
//    case Junction::RIGHT:
//      inchForward();
//      turnAngle(-90);
//      break;
//
//    case Junction::LEFT:
//      inchForward();
//      turnAngle(90);
//      break;
//
//    case Junction::LINE:
//    default:
//      // Determine angular adjustment
//      // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
//      leftMotor.driveAtSpeed(baseSpeed - maxSpeedAdjustment * skew);
//      rightMotor.driveAtSpeed(baseSpeed + maxSpeedAdjustment * skew);
//  }
//
//  // Only print ocassionally
//  if (millis() % 1000 <= 1) {
//    if (leftSpeed < 0) {
//      Serial.println("Driving Backwards");
//    }
//  
//    Serial.print("Skew1 = ");
//    Serial.println(skew);
//  
//    Serial.print("Skew2 = ");
//    Serial.println(lineSensor.getSkew2());
//  
//    Serial.print("Driving w/ speeds:\t");
//    Serial.print(leftSpeed);
//    Serial.print("\t");
//    Serial.println(rightSpeed);
//  
//    Serial.print("Junction Type: ");
//    Serial.println(junctionAsString(junction));
//  
//    Serial.println();
//  }
}
