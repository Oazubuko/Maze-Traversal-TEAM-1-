#include "Motor.h"
#include <PID_v1.h>
#include <math.h>

double leftSpeedIn, leftSpeedOut, targetLeftSpeed;
double rightSpeedIn, rightSpeedOut, targetRightSpeed;

PID leftPID(&leftSpeedIn, &leftSpeedOut, &targetLeftSpeed, 0.2, 0, 0, DIRECT);
PID rightPID(&rightSpeedIn, &rightSpeedOut, &targetRightSpeed, 0.25, 0, 0, DIRECT);

Motor leftMotor(3, 2, A0, 6, 7);
Motor rightMotor(4, 5, A1, 9, 8);

constexpr int CURVE_RADIUS = 20;
constexpr int ROBOT_RADIUS_INCHES = 10;

// Implements a circle of the form
// x(t) = CURVE_RADIUS * cos(t)
// y(t) = CURVE_RADIUS * sin(t)
double dxdt(double t) {
  return -CURVE_RADIUS * sin(t);
}
double d2x_dt2(double t) {
  return -CURVE_RADIUS * cos(t);
}
double dydt(double t) {
  return CURVE_RADIUS * cos(t);
}
double d2y_dt2(double t) {
  return -CURVE_RADIUS * sin(t);
}


/**
 * Calculates the desired forward velocity at a given time from the equation:
 * v = sqrt((x')^2 + (y')^2)
 */
double forwardSpeed(double t) {
  return sqrt(pow(dxdt(t), 2) + pow(dydt(t), 2));
}

/**
 * Calculates the desired angular velocity, given by the equation:
 * w = (y'' * x' - y' * x'') / ((x')^2 + (y')^2)
 */
double angularSpeed(double t) {
  double yPrime = dydt(t);
  double xPrime = dxdt(t);

  return (d2y_dt2(t) * xPrime - yPrime * d2x_dt2(t)) / (pow(xPrime, 2) + pow(yPrime, 2));
}

void setup() {
  leftMotor.begin();
  rightMotor.begin();

  leftPID.SetOutputLimits(-0.3, 0.3);
  rightPID.SetOutputLimits(-0.3, 0.3);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
}

double prevT = 0;
double prevLeftPos = 0;
double prevRightPos = 0;
void loop() {
  // Update setpoint
  double t = micros() * 1e-6;
  // double v = forwardSpeed(t);
  // double w = angularSpeed(t);
  // targetLeftSpeed = v + w * ROBOT_RADIUS_INCHES;
  // targetRightSpeed = v - w * ROBOT_RADIUS_INCHES;
  targetRightSpeed = 0; // Inches / sec
  targetLeftSpeed = 0.5; // Inches / sec

  // Update speed measurements
  double dt = t - prevT;
  prevT = t;

  double leftPos = leftMotor.getInchesDriven();
  double rightPos = rightMotor.getInchesDriven();
  leftSpeedIn = (leftPos - prevLeftPos) / dt;
  rightSpeedIn = (rightPos - prevRightPos) / dt;
  prevLeftPos = leftPos;
  prevRightPos = rightPos;

  // Drive motors at their PID values
  leftPID.Compute();
  rightPID.Compute();

  leftMotor.driveAtSpeed(leftSpeedOut);
  rightMotor.driveAtSpeed(rightSpeedOut);

  if (micros() % 500 == 0) {
    Serial.print("TARGET SPEED:\t");
    Serial.print(targetLeftSpeed);
    Serial.print('\t');
    Serial.println(targetRightSpeed);

    Serial.print("CURRENT SPEED:\t");
    Serial.print(leftSpeedOut);
    Serial.print('\t');
    Serial.println(rightSpeedOut);

    Serial.print("MOTOR INPUT:\t");
    Serial.print(leftSpeedIn);
    Serial.print('\t');
    Serial.println(rightSpeedIn);

    Serial.println();
  }
}