/**
 * A velocity-based controller for a motor. Takes speeds in inches / second and
 * determines the appropriate position.
 */

#ifndef MOTOR_VELOCITY_CONTROLLER
#define MOTOR_VELOCITY_CONTROLLER

#include <Arduino.h>
#include <PID_v1.h>
#include <math.h>
#include "Constants.h"
#include "PIDHelpers.h"
#include "Motor.h"
#include "MotorPositionController.h"
#include "Stopwatch.h"

class MotorVelocityController {
  private:
    Motor& _motor;
    MotorPositionController& _posController;
    double _sensedVelocity, _positionCommand, _targetVelocity;
    double _pastInchesDriven;
    Stopwatch _setpointTimer;
    PID _controller;
    
    
  public:
    MotorVelocityController(Motor& motor, MotorPositionController& posController, const PIDConstants& coeffs) :
      _motor(motor),
      _posController(posController),
      _controller(&_sensedVelocity, &_positionCommand, &_targetVelocity, coeffs.kp, coeffs.ki, coeffs.kd, DIRECT),
      _setpointTimer()
    {
      _controller.SetMode(AUTOMATIC);
    }
    
    void setTargetVelocity(double inchesPerSec) {
      _targetVelocity = inchesPerSec;
      _setpointTimer.zeroOut();
    }

    void update() {
      double dt = _setpointTimer.lap();
      double inchesDriven = _motor.getInchesDriven();
      _motorPositionController.update();
      _sensedVelocity = (inchesDriven - _prevInchesDriven) / dt;
      _controller.Compute();
    }

    void adjustPIDConstants() {
      PIDHelpers::adjustPIDConstants(_controller);
    }

    void print() {
      Serial.print("Motor Velocity PID:  ");
      Serial.print(_motor.getInchesDriven() / _setpointTimer.getElapsedTime());
      Serial.print("in. / sec. -> ");
      Serial.print(_targetVelocity);
      Serial.println(" in. / sec.");
    }
};

#endif
