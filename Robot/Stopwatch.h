#pragma once
#include <Arduino.h>

class Stopwatch {
  private:
    double _prevTimeMicros;
    double _baseTimeMicros;

  public:
    Stopwatch() {
      zeroOut();
    }
  
    /**
     * Resets the timer to zero. Should be called at least once before any other timing methods
     */
    void zeroOut() {
      _baseTimeMicros = micros();
      _prevTimeMicros = _baseTimeMicros;
    }

    /**
     * Returns the amount of time since this function or the zeroOut function was last called
     */
    double lap() {
      double now = micros();
      double timeDelta = (now - _prevTimeMicros) * 1e-6;
      _prevTimeMicros = now;

      return timeDelta;
    }

    /**
     * Return the number of seconds since the last lap, without updating any internal state
     */
    double peekTimeSinceLastLap() {
      return (micros() - _prevTimeMicros) * 1e-6;
    }

    /**
     * Returns the amount of time since the stopwatch was reset (or initialized)
     */
    double getElapsedTime() {
      return (micros() - _baseTimeMicros) * 1e-6;
    }
};
