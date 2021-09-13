/**
 * Utility class for motors and their associated sensors
 */

#ifndef CONSTANTS
#define CONSTANTS

// Calculating Encoder Ticks -> Inches
constexpr float WHEEL_DIAMETER_INCHES = 1.5;
constexpr int TICKS_PER_REV = 360;
constexpr float INCHES_PER_REV = PI * WHEEL_DIAMETER_INCHES;
constexpr int GEAR_RATIO = 30 / 1;
constexpr float ENCODER_TICKS_TO_INCHES = INCHES_PER_REV / TICKS_PER_REV;

#endif
