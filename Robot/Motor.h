/**
   Utility class for motors and their associated sensors
*/
#pragma once

#include <cstdint>
#include <Arduino.h>
#include <Encoder.h>
#include "Constants.h"

class Motor {
  private:
    uint8_t _forwardPort;
    uint8_t _backwardPort;
    Encoder _encoder;
    uint8_t _pwmSpeedLimit;
    
  public:
    static constexpr uint8_t MAX_POSSIBLE_PWM_SPEED = 255;

    /**
       @param forwardPort the output pin to drive the motor forwards
       @param backwardPort the output pin to drive the motor backwards
       @param encoderPin1 the 1st encoder pin
       @param encoderPin2 the 2nd encoder pin
       @param speedLimit limit on how fast the motor can operate as ratio in the range [0, 1].
                         0 is a speed limit of 0 mph, 1 is full speed
    */
    Motor(uint8_t forwardPort, uint8_t backwardPort, uint8_t encoderPin1, uint8_t encoderPin2, float speedLimitRatio = 1.0)
      : _forwardPort(forwardPort),
        _backwardPort(backwardPort),
        _encoder(encoderPin1, encoderPin2),

        // Limit the speed limit to the range 0 - MAX_POSSIBLE_PWM_SPEED
        _pwmSpeedLimit(max(0, min(MAX_POSSIBLE_PWM_SPEED, speedLimitRatio * MAX_POSSIBLE_PWM_SPEED)))
    {}

    /**
       You should call `begin()` in the Arduino `setup()` function to properly initialize the motor's pins
    */
    void begin() {
      pinMode(_forwardPort, OUTPUT);
      pinMode(_backwardPort, OUTPUT);
    }

    void driveForward(uint8_t pwmSpeed) {
      analogWrite(_forwardPort, min(_pwmSpeedLimit, pwmSpeed));
      analogWrite(_backwardPort, 0);
    }

    void driveBackward(uint8_t pwmSpeed) {
      analogWrite(_forwardPort, 0);
      analogWrite(_backwardPort, min(_pwmSpeedLimit, pwmSpeed));
    }

    void stop() {
      analogWrite(_forwardPort, 0);
      analogWrite(_backwardPort, 0);
    }

    /**
       Takes a speed from [-1, 1] where -1 is full speed backwards and
       1 is full speed forwards.
    */
    void driveAtSpeed(float speed) {
      int pwmSpeed = speed * _pwmSpeedLimit;

      if (pwmSpeed < 0) {
        driveBackward(-pwmSpeed); // driveBackward needs a positive value
      } else if (pwmSpeed > 0) {
        driveForward(pwmSpeed);
      } else {
        stop();
      }
    }

    int getEncoderTicks() {
      return _encoder.read();
    }

    /**
     * Transforms encoder ticks to inches
     */
    float getInchesDriven() {
      return _encoder.read() / ENCODER_TICKS_PER_INCH;
    }

    void resetEncoder() {
      _encoder.write(0);
    }
};
