/**
 * A PWM-based position controller for a motor
 */
#pragma once

#include <Arduino.h>
#include <PID_v1.h>
#include <math.h>
#include "Constants.h"
#include "PIDHelpers.h"
#include "Motor.h"
#include "Stopwatch.h"

class MotorPositionController {
  private:
    Motor& _motor;
    double _sensedPosition, _motorPWMVal, _targetPosition;
    Stopwatch _setpointTimer;
    PID _controller;
    
  public:
    MotorPositionController(Motor& motor, const PIDConstants& coeffs) :
      _motor(motor),
      _controller(&_sensedPosition, &_motorPWMVal, &_targetPosition, coeffs.kp, coeffs.ki, coeffs.kd, DIRECT),
      _setpointTimer()
    {
      _controller.SetMode(AUTOMATIC);
      _controller.SetOutputLimits(-0.3, 0.3); // This is fast enough :)
      _controller.SetSampleTime(PID_SAMPLE_PERIOD_MS);
    }
    
    void setTargetPosition(double targetPositionInInches) {
      _targetPosition = targetPositionInInches;
    }
    
    void incrementTargetPosition(double positionIncrement) {
      setTargetPosition(_targetPosition + positionIncrement);
    }

    void reset() {
      _motor.resetEncoder();
      _setpointTimer.zeroOut();
      
      setTargetPosition(0);

      // Hack to reset the output of the PID controller to 0
      // See https://github.com/br3ttb/Arduino-PID-Library/issues/76#issuecomment-678644330
      _controller.SetMode(MANUAL);
      _motorPWMVal = 0;
      _controller.SetMode(AUTOMATIC);
    }

    /**
     * Adjusts the motor's speed based on calculations of the internal PID controller.
     */
    void update() {
      // _motorPWMVal is updated by the call to .Compute()
      _sensedPosition = _motor.getInchesDriven();
      _controller.Compute();
      _motor.driveAtSpeed(_motorPWMVal);

      // If the setpoint is far away, the motor hasn't settled
      if (fabs(_sensedPosition - _targetPosition) > POSITION_THRESHOLD_INCHES) {
        _setpointTimer.zeroOut();
      }
    }

    bool reachedSetpoint() {
      return _setpointTimer.getElapsedTime() >= SETTLING_TIME;
    }

    void adjustPIDConstants() {
      PIDHelpers::adjustPIDConstants(_controller);
    }

    double getTargetPosition() {
      return _targetPosition;
    }

    void print() {
      Serial.println("Position Controller: " + String(_motor.getInchesDriven()) + " in. -> " + String(_targetPosition) + " in."  + " @ PWM " + String(_motorPWMVal));
    }
};
