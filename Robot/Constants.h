#pragma once

#include <cstdint>
#include <Arduino.h>
#include <Buzzer.h>

// Preprocessor modes
#define NO_BATTERY // Comment this line if the battery is inserted into the robot

// Misc. Constants
constexpr uint16_t PRINT_DELAY_MS = 500;
constexpr float ENCODER_TICKS_PER_INCH = 84.91667; // TODO: improve

// Pins
constexpr uint8_t BUZZER_PIN = 10;

// PID Constansts
typedef struct PIDConstants_t {
  float kp = 0;
  float ki = 0;
  float kd = 0;
} PIDConstants;

#ifndef NO_BATTERY
constexpr PIDConstants LEFT_MOTOR_POSITION_CONSTANTS = { 0.125, 0.03, 0.01 };
constexpr PIDConstants RIGHT_MOTOR_POSITION_CONSTANTS = { 0.11, 0.03, 0.0075 };
constexpr PIDConstants TURN_CONSTANTS = { 0.075, 0, 0.0075 };
constexpr float SKEW_ADJUSTMENT_FACTOR = 4;
constexpr float ANGLE_ADJUSTMENT_FACTOR = 0.2;
constexpr float SPEED_THROTTLE = 0.45; // % of max motor speed
constexpr float BASE_SPEED = 15; // Inches / sec
constexpr float MAX_TURN_SPEED = 15; // Inches / sec
#else
constexpr PIDConstants LEFT_MOTOR_POSITION_CONSTANTS = { 0.4, 0, 0.005 };
constexpr PIDConstants RIGHT_MOTOR_POSITION_CONSTANTS = { 0.4, 0, 0.005 };
constexpr PIDConstants TURN_CONSTANTS = { 0.175, 0, 0.00125 };
constexpr float SKEW_ADJUSTMENT_FACTOR = 7;
constexpr float ANGLE_ADJUSTMENT_FACTOR = 0.1;
constexpr float SPEED_THROTTLE = 0.45; // % of max motor speed
constexpr float BASE_SPEED = 13; // Inches / sec
constexpr float MAX_TURN_SPEED = 13; // Inches / sec
#endif

constexpr int PID_SAMPLE_PERIOD_MS = 10;
constexpr float DISTANCE_THRESHOLD_INCHES = 0.25;
constexpr float DEGREE_THRESHOLD = 1;
constexpr float SETTLING_TIME = 0.5; // Seconds
constexpr float PID_TIMEOUT = 5; // Seconds

// Robot Dimensions
constexpr float ROBOT_HEIGHT_INCHES = 3; // Distance from line sensor to wheels
constexpr float ROBOT_RADIUS_INCHES = 1.5625;
