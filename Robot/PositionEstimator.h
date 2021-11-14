#pragma once

#include <Arduino.h>
#include "Gyro.h"
#include "Motor.h"
#include "Constants.h"

/**
 * Estimates the (x, y, angle) position of the robot using a first-order approximation.
 * The robot's start point is considered its origin
 */
class PositionEstimator {
private:
  Motor& _leftMotor;
  Motor& _rightMotor;
  Gyro& _gyro;
  Stopwatch _stopwatch;

  float _prevLeftInches;
  float _prevRightInches;

  float _x;
  float _y;

public:
  PositionEstimator(Motor& leftMotor, Motor& rightMotor, Gyro& gyro) :
    _leftMotor(leftMotor),
    _rightMotor(rightMotor),
    _gyro(gyro),
    _stopwatch(),
    _prevLeftInches(0),
    _prevRightInches(0),
    _x(0),
    _y(0)
  {}

  /**
   * Updates the current estimate of the position. Uses a first-order (Euler) approximation
   * and vector addition to estimate the current location.
   * Should be called frequently for accurate tracking!!!
   */
  void update() {
    float leftInches = _leftMotor.getInchesDriven();
    float rightInches = _rightMotor.getInchesDriven();
    float headingRadians = _gyro.getAngle() * PI / 180.;

    // We can derive the linear velocity of the robot, v, using the following 
    // differential drive equations:
    // v_left             = v + rw
    // v_right            = v - rw
    // (v_left + v_right) = 2v
    // v                  = 1/2 * (v_left + v_right)
    // dpos = 1/2 * (v_left + v_right) * dt
    //      = 1/2 * (d_left_motor_pos + d_right_motor_pos)
    float dpos = 0.5 * ((leftInches - _prevLeftInches) + (rightInches - _prevRightInches));

    // Compute dx & dt
    _x += dpos * sin(-headingRadians);
    _y += dpos * cos(headingRadians);

    _prevLeftInches = leftInches;
    _prevRightInches = rightInches;
  }

  /**
   * Return the current guess for the robot's x position in inches
   */
  float getX() {
    return _x;
  }

  /**
   * Return the current guess for the robot's y position in inches
   */
  float getY() {
    return _y;
  }

  /**
   * Return the current guess for the robot's angular heading in degrees
   */
  float getHeading() {
    return _gyro.getAngle();
  }

  void print() {
    Serial.println("Point Estimator<x=" + String(getX()) + ", y=" + String(getY()) + ", angle=" + String(getHeading()) + ">");
  }
};
