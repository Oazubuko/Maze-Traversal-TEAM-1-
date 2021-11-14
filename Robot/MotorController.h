#pragma once

#include <Arduino.h>
#include <PID_v1.h>
#include <math.h>
#include "Constants.h"
#include "PIDHelpers.h"
#include "Motor.h"
#include "Stopwatch.h"

enum class ControlMode {
   POSITION,
   VELOCITY
};

/**
 * A motor controller with positional and velocity control modes
 */
class MotorController {
  private:
    Motor& _motor;
    PID _controller;
    Stopwatch _setpointTimer;
    ControlMode _mode = ControlMode::VELOCITY;
    double _sensedPosition = 0
    double _motorPWMVal = 0;
    double _targetPosition = 0
    double _targetVelocity = 0;
    
  public:
    MotorController(Motor& motor, const PIDConstants& coeffs) :
      _motor(motor),
      _controller(&_sensedPosition, &_motorPWMVal, &_targetPosition, coeffs.kp, coeffs.ki, coeffs.kd, DIRECT),
      _setpointTimer()
    {
      _controller.SetMode(AUTOMATIC);
      _controller.SetOutputLimits(-SPEED_THROTTLE, SPEED_THROTTLE);
      _controller.SetSampleTime(PID_SAMPLE_PERIOD_MS);
    }

    void setTargetPosition(double targetPositionInInches) {
      _targetPosition = targetPositionInInches;
    }

    void setTargetVelocity(double targetVelocity) {
       _targetVelocity = targetVelocity;
    }

   /**
    * Adjust the control modes, which determines whether to ramp the setpoint based
    * on a target velocity or to set a static positional setpoint
    */
    void setControlMode(ControlMode newMode) {
      _mode = newMode;
    }

    /**
     * Adjusts the motor's speed based on calculations of the internal PID controller
     */
    void update() {
       if (_mode == ControlMode::VELOCITY) {
         rampTargetPosition();
       }

      // _motorPWMVal is updated by the call to .Compute()
      _sensedPosition = _motor.getInchesDriven();
      _controller.Compute();
      _motor.driveAtSpeed(_motorPWMVal);

      // If the setpoint is far away, the motor hasn't settled
      if (fabs(_sensedPosition - _targetPosition) > POSITION_THRESHOLD_INCHES) {
        _setpointTimer.zeroOut();
      }
    }

    void reset() {
      setTargetPosition(0);
      resetPIDOutput();
      _motor.resetEncoder();
      _setpointTimer.zeroOut();
    }

    bool reachedSetpoint() {
      return _setpointTimer.getElapsedTime() >= SETTLING_TIME;
    }

   /**
    * Opens a Serial-based dialog to adjust the PID constants of the motor controller
    */
    void adjustPIDConstants() {
      PIDHelpers::adjustPIDConstants(_controller);
    }

    double getTargetPosition() {
      return _targetPosition;
    }

    double getTargetVelocity() {
       return _targetVelocity;
    }

    void print() {
      Serial.println("Motor Controller: " + String(_motor.getInchesDriven()) + " in. -> " + String(_targetPosition) + " in."  + " @ PWM " + String(_motorPWMVal));
    }

// Helper functions
private:
    /**
     * Hack to reset the output of the PID controller to 0 and prevent
     * build-up of the integral term between separate commands.
     * See https://github.com/br3ttb/Arduino-PID-Library/issues/76#issuecomment-678644330
     */
    void resetPIDOutput() {
      _controller.SetMode(MANUAL);
      _motorPWMVal = 0;
      _controller.SetMode(AUTOMATIC);
    }

   /**
    * Ramp up the positional setpoint up based on the current target velocity and the
    * time since the last update
    */
    void rampTargetPosition() {
       double positionIncrement = _targetVelocity * _setpointTimer.lap();
       setTargetPosition(_targetPosition + positionIncrement);
    }
};
