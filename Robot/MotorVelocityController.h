/**
 * A velocity-based controller for a motor. Takes speeds in inches / second and
 * determines the appropriate position.
 */


 /**
  * Notes from Levi:
  *  - Having an outer PID controller on velocity has the same effect as adjusting the gain
  *    of the inner loop (by linearity of the P, I, and D terms)
  *  - So we only need an open-loop rule to determine the position value to pass to the
  *    positional controller
  *  - This will result in us asymptotically reaching the desired velocity!
  *  - But we will still have steady state positional error.
  *  - How do we fix that?
  *  - We can 
  *     - create a full-state feedback controller with the following state:
  *         - integral of positional error
  *         - position
  *         - velocity
  *     - OR you can integrate the positional error at the outer loop, and pass that in as an additional
  *       component as the velocity command. This allows you to maintain the separation of the two loops.
  */
#pragma once

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
    Stopwatch _setpointTimer;
    double _targetVelocity;
    double _prevInchesDriven;
    double _totalPositionalError;
    double _prevPositionIncrement;
    double _dt;
    
  public:
    MotorVelocityController(Motor& motor, MotorPositionController& posController) :
      _motor(motor),
      _posController(posController),
      _setpointTimer()
    {}
    
    void setTargetVelocity(double inchesPerSec) {
      _targetVelocity = inchesPerSec;
    }

    void reset() {
      _posController.reset();
      _setpointTimer.zeroOut();

      // Extra statistics for debugging purposes
      _prevInchesDriven = _motor.getInchesDriven();
      _totalPositionalError = 0;
    }

    /**
     * Drive the motor an extra v_target * dt every loop.
     * Should be called at the specified PID sample rate for solid performance
     */
    void update() {
      // Every loop, the motor should drive an extra 
      _dt = _setpointTimer.lap();
      _posController.incrementTargetPosition(_targetVelocity * _dt);
      _posController.update();
    }

    double getTargetVelocity() {
      return _targetVelocity;
    }

    void print() {
      // Calculate some statistics about the current velocity
      double inchesDriven = _motor.getInchesDriven();
      double inchesThisLoop = inchesDriven - _prevInchesDriven;
      _prevInchesDriven = inchesDriven;
      
      double sensedVelocity = inchesThisLoop / _dt;
      _totalPositionalError += inchesThisLoop - _prevPositionIncrement;
      _prevPositionIncrement = _targetVelocity * _dt;
      
      Serial.println("dt: " + String(_dt, 6) + " seconds");
      Serial.println("Velocity: " + String(sensedVelocity, 6) + " in. / sec -> " + String(_targetVelocity, 6) + " in. / sec");
      Serial.println("Position Increment: " + String(_prevPositionIncrement, 6) + " in.");
      Serial.println("Total Positional Error: " + String(_totalPositionalError, 6) + " in.");
      Serial.println();
    }
};
