#pragma once

#include <Arduino.h>
#include <PID_v1.h>
#include <math.h>
#include <limits.h>
#include "Constants.h"
#include "Motor.h"
#include "Stopwatch.h"
#include "PIDController.h"

enum class ControlMode {
  POSITION,
  VELOCITY
};

/**
   A motor controller with positional and velocity control modes.

   Velocity control is achieved by ramping the positional setpoint
   in small increments.
*/
class MotorController {
  private:
    Motor& _motor;
    PIDController _controller;
    Stopwatch _maxPositionTimer;
    ControlMode _mode = ControlMode::VELOCITY;
    double _maxPosition = std::numeric_limits<double>::max();
    double _targetVelocity = 0;

  public:
    MotorController(Motor& motor, const PIDConstants& coeffs) :
      _motor(motor),
      _controller(coeffs, -SPEED_THROTTLE, SPEED_THROTTLE, DISTANCE_THRESHOLD_INCHES)
    {
    }

    /**
       Sets the target position
    */
    void setTargetPosition(double targetPositionInInches) {
      _controller.SetSetpoint(targetPositionInInches);
    }

    /**
       Sets the target position (only works in VELOCITY control mode)
    */
    void setTargetVelocity(double targetVelocity) {
      _targetVelocity = targetVelocity;
    }

    /**
       Adjust the control modes, which determines whether to ramp the setpoint based
       on a target velocity or to set a static positional setpoint
    */
    void setControlMode(ControlMode newMode) {
      _mode = newMode;
    }

    /**
       Adjusts the motor's speed based on calculations of the internal PID controller
    */
    void update() {
      if (_mode == ControlMode::VELOCITY) {
        rampTargetPosition();
      }

      // TODO: handle error where angular error causes the max position to never be reached.
      // I'm not sure if this is possible in practice with a well-charged battery.
      if (_controller.GetSetpoint() < _maxPosition) {
        _maxPositionTimer.zeroOut();
      }

      double motorPWMValue = _controller.Compute(_motor.getInchesDriven());
      _motor.driveAtSpeed(motorPWMValue);
    }

    /**
       Reset the state of the controller (i.e., the setpoint and outputs). Does not modify the
       control mode.
    */
    void reset() {
      _motor.stop();
      _controller.Reset();
      _maxPositionTimer.zeroOut();
      _motor.resetEncoder();
      _maxPosition = std::numeric_limits<double>::max();
    }

    bool reachedSetpoint() {
      return _controller.ReachedSetpoint();
    }

    bool reachedMaxPosition() {
      return reachedSetpoint() && _maxPositionTimer.getElapsedTime() >= SETTLING_TIME;
    }

    double getTargetPosition() {
      return _controller.GetSetpoint();
    }

    double getTargetVelocity() {
      return _targetVelocity;
    }

    /**
       Set the maximum position setpoint. This parameter can be used to achieve a target
       positional setpoint in velocity control mode
    */
    void setMaxPosition(double maxPosition) {
      _maxPosition = maxPosition;
      _maxPositionTimer.zeroOut();
    }

    void print() {
      Serial.print("Motor ");
      _controller.Print();
    }

    void adjustPIDConstants() {
      _controller.AdjustPIDConstants();
    }

    // Helper functions
  private:
    /**
       Ramp up the positional setpoint up based on the current target velocity and the
       time since the last PID computation
    */
    void rampTargetPosition() {
      double positionIncrement = _targetVelocity * _controller.GetTimeDelta();
      double nextPositionSetpoint = _controller.GetSetpoint() + positionIncrement;

      _controller.SetSetpoint(min(nextPositionSetpoint, _maxPosition));
    }
};
