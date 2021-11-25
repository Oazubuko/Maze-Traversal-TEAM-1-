#pragma once

#include <cstdint>
#include <Arduino.h>

// Misc. Constants
constexpr uint16_t PRINT_DELAY_MS = 500;

// Pins
constexpr uint8_t BUZZER_PIN = 10;

// Calculating Encoder Ticks -> Inches
constexpr float ENCODER_TICKS_PER_INCH = 84.91667; // TODO: improve
constexpr float POSITION_THRESHOLD_INCHES = 1;

// PID Constansts
typedef struct PIDConstants_t {
  float kp = 0;
  float ki = 0;
  float kd = 0;
} PIDConstants;
constexpr PIDConstants LEFT_MOTOR_POSITION_CONSTANTS = { 0.125, 0.01, 0.01 };
constexpr PIDConstants RIGHT_MOTOR_POSITION_CONSTANTS = { 0.11, 0.01, 0.0075 };
constexpr PIDConstants TURN_CONSTANTS = { 0.075, 0, 0 };
constexpr int PID_SAMPLE_PERIOD_MS = 5;
constexpr float DISTANCE_THRESHOLD_INCHES = 0.5;
constexpr float DEGREE_THRESHOLD = 2;
constexpr float SETTLING_TIME = 0.25; // Seconds
constexpr float PID_TIMEOUT = 1; // Seconds

constexpr float SKEW_ADJUSTMENT_FACTOR = 4;
constexpr float ANGLE_ADJUSTMENT_FACTOR = 0.2;

// Speeds
constexpr float SPEED_THROTTLE = 1; // % of max motor speed
constexpr float BASE_SPEED = 13; // Inches / sec
constexpr float MAX_TURN_SPEED = 20; // Inches / sec

// Robot Dimensions
constexpr float ROBOT_HEIGHT_INCHES = 3; // Distance from line sensor to wheels
constexpr float ROBOT_RADIUS_INCHES = 1.5625;
