/**
 * Utility class for motors and their associated sensors
 */
#ifndef CONSTANTS
#define CONSTANTS

#include <cstdint>
#include <Arduino.h>

// Calculating Encoder Ticks -> Inches
constexpr float WHEEL_DIAMETER_INCHES = 1.5;
constexpr int TICKS_PER_REV = 360;
constexpr float INCHES_PER_REV = PI * WHEEL_DIAMETER_INCHES;
constexpr int GEAR_RATIO = 30 / 1;
constexpr float ENCODER_TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;

typedef struct PIDConstants_t {
  float kp = 0;
  float ki = 0;
  float kd = 0;
} PIDConstants;

#endif
