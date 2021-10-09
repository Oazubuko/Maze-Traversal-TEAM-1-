#ifndef CONSTANTS
#define CONSTANTS

#include <cstdint>
#include <Arduino.h>
#include "PIDHelpers.h"

// Calculating Encoder Ticks -> Inches
constexpr float WHEEL_DIAMETER_INCHES = 1.5;
constexpr int TICKS_PER_REV = 360;
constexpr float INCHES_PER_REV = PI * WHEEL_DIAMETER_INCHES;
constexpr int GEAR_RATIO = 30 / 1;
constexpr float ENCODER_TICKS_PER_INCH = TICKS_PER_REV / INCHES_PER_REV;
constexpr float POSITION_THRESHOLD_INCHES = 1;

// PID Constansts
constexpr PIDConstants LEFT_MOTOR_POSITION_CONSTANTS = { 0.125, 0.01, 0.01 };
constexpr PIDConstants RIGHT_MOTOR_POSITION_CONSTANTS = { 0.11, 0.01, 0.0075 };
constexpr PIDConstants LEFT_MOTOR_VELOCITY_CONSTANTS = { 0.5, 0, 0 };
constexpr PIDConstants distanceConstants = { 0.075, 0, 0 };
constexpr PIDConstants turningConstants = { 0.2, 0, 0 };

constexpr float DISTANCE_THRESHOLD_INCHES = 0.8;
constexpr float DEGREE_THRESHOLD = 4;
constexpr float SETTLING_TIME = 0.5;

// Gyro Constants
constexpr float GYRO_BIAS = 0.74; // Degrees / sec

#endif
