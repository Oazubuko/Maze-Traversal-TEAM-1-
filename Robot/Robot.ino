#include <Arduino_LSM9DS1.h>
#include <PID_v1.h>
#include <Buzzer.h>
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"
#include "MotorPositionController.h"
#include "MotorVelocityController.h"
#include "Songs.h"

Buzzer buzzer(10);

Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);

MotorPositionController leftPosController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorPositionController rightPosController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);

MotorVelocityController leftVelocityController(leftMotor, leftPosController);
MotorVelocityController rightVelocityController(rightMotor, rightPosController);

constexpr double CURVE_RADIUS = 20;
constexpr double ROBOT_RADIUS_INCHES = 1.5625;
constexpr double ANGULAR_SPEED = 0.75;

// Implements a circle of the form
// x(t) = CURVE_RADIUS * cos(ANGULAR_SPEED * t)
// y(t) = CURVE_RADIUS * sin(ANGULAR_SPEED * t)
//double dxdt(double t) {
//  return -CURVE_RADIUS * ANGULAR_SPEED * sin(ANGULAR_SPEED * t);
//}
//double d2x_dt2(double t) {
//  return -CURVE_RADIUS * Utils::square(ANGULAR_SPEED) * cos(ANGULAR_SPEED * t);
//}
//double dydt(double t) {
//  return CURVE_RADIUS * ANGULAR_SPEED * cos(ANGULAR_SPEED * t);
//}
//double d2y_dt2(double t) {
//  return -CURVE_RADIUS * Utils::square(ANGULAR_SPEED) * sin(ANGULAR_SPEED * t);
//}

// Implements a figure 8 of the form
// x(t) = CURVE_RADIUS * sin(ANGULAR_SPEED * t)
// y(t) = CURVE_RADIUS / 2 * sin(2 * ANGULAR_SPEED * t)
double dxdt(double t) {
  return CURVE_RADIUS * ANGULAR_SPEED * cos(ANGULAR_SPEED * t);
}
double d2x_dt2(double t) {
  return -CURVE_RADIUS * Utils::square(ANGULAR_SPEED) * sin(ANGULAR_SPEED * t);
}
double dydt(double t) {
  return CURVE_RADIUS * ANGULAR_SPEED * cos(2 * ANGULAR_SPEED * t);
}
double d2y_dt2(double t) {
  return -2 * CURVE_RADIUS * Utils::square(ANGULAR_SPEED) * sin(2 * ANGULAR_SPEED * t);
}

/**
   Calculates the desired forward velocity at a given time from the equation:
   v = sqrt((x')^2 + (y')^2)
*/
double forwardSpeed(double t) {
  return sqrt(Utils::square(dxdt(t)) + Utils::square(dydt(t)));
}

/**
   Calculates the desired angular velocity, given by the equation:
   w = (y'' * x' - y' * x'') / ((x')^2 + (y')^2)
*/
double angularSpeed(double t) {
  double yPrime = dydt(t);
  double xPrime = dxdt(t);

  return (d2y_dt2(t) * xPrime - yPrime * d2x_dt2(t)) / (Utils::square(xPrime) + Utils::square(yPrime));
}

void setup() {
  Serial.begin(9600);
  Songs::playStarWarsTheme(buzzer);

  leftVelocityController.reset();
  rightVelocityController.reset();
}

void loop() {
  double t = micros() * 1e-6;
  double v = forwardSpeed(t);
  double w = angularSpeed(t);
  double targetLeftSpeed = v + w * ROBOT_RADIUS_INCHES;
  double targetRightSpeed = v - w * ROBOT_RADIUS_INCHES;
  
  leftVelocityController.setTargetVelocity(targetLeftSpeed);
  rightVelocityController.setTargetVelocity(targetRightSpeed);

  leftVelocityController.update();
  rightVelocityController.update();

  Serial.println("LEFT");
  leftPosController.print();
  Serial.println("RIGHT");
  rightPosController.print();
  
  delay(PID_SAMPLE_PERIOD_MS);
}
