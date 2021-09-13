/**
 * Utility class for motors and their associated sensors
 */

#ifndef MOTOR
#define MOTOR

#include <cstdint>
#include <Arduino.h>
#include <Encoder.h>
#include "Constants.h"

// TODO: create motor pin struct

class Motor {
  private:
    static constexpr float CURRENT_SENSOR_TICKS_TO_AMPS = (3.3 / 1024.0) / 0.120;
    uint8_t _forwardPort;
    uint8_t _backwardPort;
    uint8_t _currentPort;
    uint8_t _pwmSpeedLimit;
    Encoder _encoder;

  public:
    static constexpr uint8_t MAX_POSSIBLE_PWM_SPEED = 255;
  
    /**
     * @param forwardPort the output pin to drive the motor forwards
     * @param backwardPort the output pin to drive the motor backwards
     * @param currentPort the analog pin sense the current
     * @param encoderPin1 the 1st encoder pin
     * @param encoderPin2 the 2nd encoder pin
     * @param speedLimit limit on how fast the motor can operate as ratio in the range [0, 1].
     *                   0 is a speed limit of 0 mph, 1 is full speed
     */
    Motor(uint8_t forwardPort, uint8_t backwardPort, uint8_t currentPort, uint8_t encoderPin1, uint8_t encoderPin2, float speedLimitRatio=1.0)
      : _forwardPort(forwardPort),
        _backwardPort(backwardPort),
        _currentPort(currentPort),
        _encoder(encoderPin1, encoderPin2),
        _pwmSpeedLimit(max(0, min(MAX_POSSIBLE_PWM_SPEED, speedLimitRatio * MAX_POSSIBLE_PWM_SPEED))) // Limit the speed limit to the range 0 - MAX_POSSIBLE_PWM_SPEED
    {}

    /**
     * You should call `begin()` in the Arduino `setup()` function to properly initialize the motor's pins
     */
    void begin() {
      pinMode(_forwardPort, OUTPUT);
      pinMode(_backwardPort, OUTPUT);
      pinMode(_currentPort, INPUT);
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
      * Takes a speed from [-1, 1] where -1 is full speed backwards and
      * 1 is full speed forwards.
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

    float getCurrentDrawAmps() {
      return CURRENT_SENSOR_TICKS_TO_AMPS * analogRead(_currentPort);
    }


  /**
   * Transforms encoder ticks to inches
   */
    float getInchesDriven() {
      return ENCODER_TICKS_TO_INCHES * _encoder.read();
    }

    void resetSensors() {
      _encoder.write(0);
    }
};

#endif
