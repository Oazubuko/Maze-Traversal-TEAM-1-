#ifndef STOPWATCH
#define STOPWATCH

#include <Arduino.h>

class Stopwatch {
  private:
    double _prevTimeMicros;

  public:
    /**
     * Resets the timer to zeroOut should be called at least once before any other timing methods
     */
    void zeroOut() {
      _prevTimeMicros = micros();
    }

    /**
     * Returns the amount of time since this function or the zeroOut function was last called and
     * updates the base time
     */
    double lap() {
      double now = micros();
      double timeDelta = (now - _prevTimeMicros) * 1e-6;
      _prevTimeMicros = now;

      return timeDelta;
    }

    /**
     * Returns the amount of time spent **without updating the base time**
     */
    double getElapsedTime() {
      return (micros() - _prevTimeMicros) * 1e-6;
    }
};

#endif
