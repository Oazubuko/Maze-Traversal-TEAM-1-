/**
 * A PWM-based position controller for a motor
 */

#ifndef MOTOR_POS_CONTROLLER
#define MOTOR_POS_CONTROLLER

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

      // Don't let the motor speed exceed 30% of max
      _controller.SetOutputLimits(-0.3, 0.3);
    }
    
    void setTargetPosition(double targetPositionInInches) {
      _targetPosition = targetPositionInInches;
      _setpointTimer.zeroOut();
    }

    void incrementTargetPosition(double positionIncrementInInches) {
      setTargetPosition(positionIncrementInInches + _targetPosition);
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

    void print() {
      Serial.println("Position Controller: " + String(_motor.getInchesDriven()) + " in. -> " + String(_targetPosition) + " in.");
    }
};

#endif
