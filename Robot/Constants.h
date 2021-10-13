#ifndef CONSTANTS
#define CONSTANTS

#include <cstdint>
#include <Arduino.h>
#include "PIDHelpers.h"

// Calculating Encoder Ticks -> Inches
constexpr float ENCODER_TICKS_PER_INCH = 84.91667;
constexpr float POSITION_THRESHOLD_INCHES = 1;

// PID Constansts
constexpr PIDConstants LEFT_MOTOR_POSITION_CONSTANTS = { 0.125, 0.01, 0.01 };
constexpr PIDConstants RIGHT_MOTOR_POSITION_CONSTANTS = { 0.11, 0.01, 0.0075 };
constexpr PIDConstants TURN_CONSTANTS = { 0.075, 0, 0 };
constexpr int PID_SAMPLE_PERIOD_MS = 20;
constexpr float DISTANCE_THRESHOLD_INCHES = 0.8;
constexpr float DEGREE_THRESHOLD = 1;
constexpr float SETTLING_TIME = 0.5; // Seconds

// Gyro Constants
constexpr float GYRO_BIAS = 0.74; // Degrees / sec

// Speeds
constexpr float BASE_SPEED = 5; // Inches / sec
constexpr float MAX_SPEED_ADJUSTMENT = 0.5;

// Robot Dimensions
constexpr float ROBOT_HEIGHT_INCHES = 3; // Distance from line sensor to wheels
constexpr float ROBOT_RADIUS_INCHES = 1.5625;

#endif
