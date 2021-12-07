#pragma once

#include <cstdint>
#include <Arduino.h>
#include <Buzzer.h>

/**
 * Preprocessor modes
 */
//#define NO_BATTERY                  // Comment this line if the battery is inserted into the robot
#define TAPE_CUTOFF_REFLECTANCE 690   // Comment this line to enable automatic tape cutoff detection for the line sensor

/**
 * Mouse Owner variables (used for mouse-specific configuration)
 */
//#define ZACHS_MOUSE             // Comment this line if you're not Zach
#define MACHIS_MOUSE          // Comment this line if you're not Machi
//#define EDS_MOUSE             // Comment this line if you're not Ed

/**
 * Bluetooth
 */
const char MAZE_SERVICE_ID[] = "0fe79935-cd39-480a-8a44-06b70f36f248";
const char NO_DIRECTIONS[] = "None";
enum FlagCharacteristicVals { // TODO figure out what A, B, and C are!
  A = -1,
  B = 1,
  C = 2,
  D = 4
};

#if defined(EDS_MOUSE)
const char FLAG_CHARACTERISTIC_ID[] = "0fe79935-cd39-480a-8a44-06b70f36f24a";
const char DIRECTIONS_CHARACTERISTIC_ID[] = "0fe79935-cd39-480a-8a44-06b70f36f24c";
const char MOUSE_NAME[] = "Ed's Mouse 🐢";

#elif defined(ZACHS_MOUSE)
const char FLAG_CHARACTERISTIC_ID[] = "1fe79935-cd39-480a-8a44-06b70f36f24a";
const char DIRECTIONS_CHARACTERISTIC_ID[] = "1fe79935-cd39-480a-8a44-06b70f36f24c";
const char MOUSE_NAME[] = "Zach's Mouse 🐳";

#elif defined(MACHIS_MOUSE)
const char FLAG_CHARACTERISTIC_ID[] = "2fe79935-cd39-480a-8a44-06b70f36f24a";
const char DIRECTIONS_CHARACTERISTIC_ID[] = "2fe79935-cd39-480a-8a44-06b70f36f24c";
const char MOUSE_NAME[] = "Machi's Mouse 🦑";

#endif

/**
 * Miscellaneous Constants
 */
constexpr uint16_t PRINT_DELAY_MS = 500;
constexpr float ENCODER_TICKS_PER_INCH = 84.91667; // TODO: improve

// Number of consecutive line readings of a given type before
// committing to a line reading.
constexpr unsigned int LINE_CONFIDENCE_THRESHOLD = 4;

/**
 * Pins
 */
constexpr uint8_t BUZZER_PIN = 10;

/**
 * PID Constants
 */
typedef struct PIDConstants_t {
  float kp = 0;
  float ki = 0;
  float kd = 0;
} PIDConstants;

#ifndef NO_BATTERY
constexpr PIDConstants LEFT_MOTOR_POSITION_CONSTANTS = { 0.3, 0.3, 0.01 };
constexpr PIDConstants RIGHT_MOTOR_POSITION_CONSTANTS = { 0.3, 0.3, 0.01 };
constexpr PIDConstants TURN_CONSTANTS = { 0.14, 0, 0 };
constexpr float SKEW_ADJUSTMENT_FACTOR = 4;
constexpr float ANGLE_ADJUSTMENT_FACTOR = 0.2;
constexpr float SPEED_THROTTLE = 0.45; // % of max motor speed
constexpr float BASE_SPEED = 8; // Inches / sec
constexpr float ID_JUNCTION_SPEED = 8; // Inches / sec
constexpr float MAX_TURN_SPEED = 13; // Inches / sec
#else
constexpr PIDConstants LEFT_MOTOR_POSITION_CONSTANTS = { 0.4, 0, 0.005 };
constexpr PIDConstants RIGHT_MOTOR_POSITION_CONSTANTS = { 0.4, 0, 0.005 };
constexpr PIDConstants TURN_CONSTANTS = { 0.175, 0, 0.00125 };
constexpr float SKEW_ADJUSTMENT_FACTOR = 7;
constexpr float ANGLE_ADJUSTMENT_FACTOR = 0.1;
constexpr float SPEED_THROTTLE = 0.45; // % of max motor speed
constexpr float BASE_SPEED = 13; // Inches / sec
constexpr float ID_JUNCTION_SPEED = 6; // Inches / sec
constexpr float MAX_TURN_SPEED = 13; // Inches / sec
#endif

constexpr int PID_SAMPLE_PERIOD_MS = 10;
constexpr float DISTANCE_THRESHOLD_INCHES = 0.25;
constexpr float DEGREE_THRESHOLD = 3;
constexpr float SETTLING_TIME = 0.35; // Seconds
constexpr float PID_TIMEOUT = 5; // Seconds

constexpr float inchToCm = 2.54; // conversion factor need to convert from inches to centimeters
constexpr float BASE_SPEED_CM = BASE_SPEED*inchToCm;
constexpr float CM_PER_UNIT = 15.0;

constexpr float UNIT_SPEED = BASE_SPEED_CM / CM_PER_UNIT;

constexpr int NUM_MAZE_ROWS = 99;
constexpr int NUM_MAZE_COLUMNS = 99;
/**
 * Robot Dimensions
 */
constexpr float ROBOT_HEIGHT_INCHES = 3; // Distance from line sensor to wheels
constexpr float ROBOT_HEIGHT_CM = ROBOT_HEIGHT_INCHES * inchToCm;
constexpr float ROBOT_RADIUS_INCHES = 1.5625;
