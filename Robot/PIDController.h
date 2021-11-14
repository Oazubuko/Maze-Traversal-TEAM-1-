#pragma once

#include <Arduino.h>
#include <PID_v1.h>
#include "Stopwatch.h"
#include "Utils.h"

/**
   PIDController class with a simplified interface. Note that UpperCamelCase
   is used to conform with the style of the original PID library
*/
class PIDController : public PID
{
  private:
    Stopwatch _setpointTimer;
    double _input = 0;
    double _output = 0;
    double _setpoint = 0;
    double _errorTolerance;

  public:
    PIDController(const PIDConstants& coeffs, double outputMin, double outputMax, double errorTolerance) :
      PID(&_input, &_output, &_setpoint, coeffs.kp, coeffs.ki, coeffs.kd, DIRECT)
    {
      PID::SetOutputLimits(outputMin, outputMax);
      PID::SetMode(AUTOMATIC);
      PID::SetSampleTime(PID_SAMPLE_PERIOD_MS);

      _errorTolerance = errorTolerance;
    }

    void SetSetpoint(double newSetpoint) {
      _setpoint = newSetpoint;
    }

    void IncrementSetpoint(double increment) {
      _setpoint += increment;
    }

    double GetSetpoint() {
      return _setpoint;
    }

    /**
       Returns the next output of the PID controller based on a new sensed value.
       MUST BE CALLED AT THE PID SAMPLE PERIOD (see Constants.h) for accurate results
    */
    double Compute(double nextInput) {
      // If the setpoint is far away, the plant hasn't settled
      if (fabs(_input - _setpoint) > _errorTolerance) {
        _setpointTimer.zeroOut();
      } else {
        _setpointTimer.lap();
      }

      // PID::Compute() automatically updates _output based on current error
      _input = nextInput;
      PID::Compute();

      return _output;
    }

    /**
       Returns how many seconds elapsed since a PID calculation was last run
    */
    double GetTimeDelta() {
      return _setpointTimer.peekTimeSinceLastLap();
    }

    /**
       Whether the plant has settled on the setpoint for long enough
    */
    bool ReachedSetpoint() {
      return _setpointTimer.getElapsedTime() >= SETTLING_TIME;
    }

    /**
       Resets the state of the PID controller (prevents build-up of the integral
       term between separate setpoints)
    */
    void Reset() {
      // Hack to prevent integral buildup
      // See https://github.com/br3ttb/Arduino-PID-Library/issues/76#issuecomment-678644330
      PID::SetMode(MANUAL);
      _output = 0;
      _input = 0;
      _setpoint = 0;
      PID::SetMode(AUTOMATIC);

      _setpointTimer.zeroOut();
    }

    void Print() {
      Serial.println("PID Controller: " + String(_input) + " -> " + String(_setpoint) + " @ " + String(_output));
    }

    /**
       Opens a dialog in the Serial monitor to adjust the PID constants of this controller.
       The new constants should be entered in the form "kp, ki, kd"
    */
    void AdjustPIDContants() {
      Serial.println("Enter your PID constants in the form 'kp,ki,kd': ");

      if (!Utils::awaitUserInput()) {
        Serial.println("Timed out while awaiting user input, keeping PID constants the same.");
        return;
      }

      String user_input = Serial.readString();

      // Parse constants kp, ki, kd
      int kp_end = user_input.indexOf(',');
      int ki_end = user_input.indexOf(',', kp_end + 1);

      double kp = user_input.substring(0, kp_end).toDouble();
      double ki = user_input.substring(kp_end + 1, ki_end).toDouble();
      double kd = user_input.substring(ki_end + 1).toDouble();

      Serial.print("Setting PID constants to: ");
      Serial.print(kp);
      Serial.print(", ");
      Serial.print(ki);
      Serial.print(", ");
      Serial.println(kd);

      PID::SetTunings(kp, ki, kd);
    }
};
