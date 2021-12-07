#pragma once

#include <cstdint>
#include <Arduino.h>
#include "PIDHelpers.h"

// Calculating Encoder Ticks -> Inches
constexpr float ENCODER_TICKS_PER_INCH = 84.91667; // TODO: improve
constexpr float POSITION_THRESHOLD_INCHES = 1;

// PID Constansts
constexpr PIDConstants LEFT_MOTOR_POSITION_CONSTANTS = { 0.125, 0.01, 0.01 };
constexpr PIDConstants RIGHT_MOTOR_POSITION_CONSTANTS = { 0.11, 0.01, 0.0075 };
constexpr PIDConstants TURN_CONSTANTS = { 0.075, 0, 0 };
constexpr int PID_SAMPLE_PERIOD_MS = 20;
constexpr float DISTANCE_THRESHOLD_INCHES = 0.8;
constexpr float DEGREE_THRESHOLD = 2;
constexpr float SETTLING_TIME = 0.5; // Seconds
constexpr float PID_TIMEOUT = 5; // Seconds

// Speeds
constexpr float BASE_SPEED = 11; // Inches / sec  13 in version 5, chnaged to 11 in version 6.1, changed to 9 in version 6.2
constexpr float SKEW_ADJUSTMENT_FACTOR = 3.5;//4 in verion 5, changed to 3 in version 6.1, changed to 2 in version 6.2
constexpr float MAX_TURN_SPEED = 20; // Inches / sec
constexpr float ANGLE_PID_RAMP_SPEED = 70; // Inches / sec

// Robot Dimensions
constexpr float ROBOT_HEIGHT_INCHES = 4; // Distance from line sensor to wheels
constexpr float ROBOT_RADIUS_INCHES = 1.5625;
