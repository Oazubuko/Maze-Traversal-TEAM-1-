#include <vector>
#include <PID_v1.h>
#include <Buzzer.h>
#include <Arduino_LSM9DS1.h>
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"
#include "MotorController.h"
#include "Songs.h"
#include "Stopwatch.h"
#include "Gyro.h"
#include "PositionEstimator.h"
#include "StateMachine.h"
#include "Errors.h"

State state = State::FOLLOWING_LINE;

// Turning FSMs
enum class TurnState {INIT, TURNING, DONE};
TurnState turnState = TurnState::INIT;

// Junction identification FSM
enum class JunctionIDState {INIT, CENTERING_ON_JUNCTION, DONE};
JunctionIDState junctionIDState = JunctionIDState::INIT;
Junction identifiedJunction;
LineReading firstLineReading;

// Global objects
LineSensor lineSensor(A3, A2);
Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);
MotorController leftMotorController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorController rightMotorController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);
Gyro gyro;
PositionEstimator posEstimator(leftMotor, rightMotor, gyro);

// Forward Declarations
void playToneFor(Junction junction, unsigned int duration=50);
int getToneFor(Junction junction);
int pickTurnDirection();
void printStatus();
void updateMotorSpeeds(double leftSpeed, double rightSpeed);
Junction determineJunction(LineReading firstReading, LineReading lastReading);

void setup() {
  Serial.begin(9600);
  Songs::playStarWarsTheme();

  gyro.begin();
  lineSensor.begin();

  leftMotorController.reset();
  rightMotorController.reset();
}

/**
 * The loop is organized as a Finite State Machine. During each loop, we:
 * 1. Read sensor values
 * 2. Transition to a new state based on the sensor readings
 * 3. Output the appropriate signals based on our current state + inputs
 */
void loop() {
  // Read sensors
  LineReading lineReading = lineSensor.getReading();
  gyro.update();
  posEstimator.update();  

  // Next State logic
  switch (state) {
    case State::FOLLOWING_LINE:
      state = followingLineNextState(lineReading);
      break;

    case State::IDENTIFYING_JUNCTION:
      state = identifyingJunctionNextState();
      break;

    case State::TURNING:
      state = turningNextState();
      break;

    case State::FINISHED:
      state = finishedNextState();
      break;

    default:
      handleFatalError("Illegal state in next state logic check");
      break;
  }

  // Output Logic
  switch (state) {
    case State::FOLLOWING_LINE:
      followingLineActions();
      break;

    case State::IDENTIFYING_JUNCTION:
      identifyingJunctionActions(lineReading);
      break;

    case State::TURNING:
      turningActions();
      break;

    case State::FINISHED:
      turningActions();
      break;

    default:
      handleFatalError("Illegal state in next state logic check");
      break;
  }

  printStatus();

  delay(PID_SAMPLE_PERIOD_MS);
}


// State transitions
State followingLineNextState(LineReading lineReading) {
  if (lineReading != LineReading::CENTERED) return State::IDENTIFYING_JUNCTION;
  
  return State::FOLLOWING_LINE;
}

State identifyingJunctionNextState() {
  if (junctionIDState == JunctionIDState::DONE) return State::TURNING;

  return State::IDENTIFYING_JUNCTION;
}

State turningNextState() {
  if (turnState == TurnState::DONE) return State::FOLLOWING_LINE;

  return State::TURNING;
}

State finishedNextState() {
  return State::FINISHED;
}

// State actions
void followingLineActions() {
  // Determine angular adjustment using the line sensor's 'skew' measurement
  // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
  float skew = lineSensor.getSkew2();
  double leftSpeed = BASE_SPEED - SKEW_ADJUSTMENT_FACTOR * skew;
  double rightSpeed = BASE_SPEED + SKEW_ADJUSTMENT_FACTOR * skew;
  updateMotorSpeeds(leftSpeed, rightSpeed);
}

void identifyingJunctionActions(LineReading latestLineReading) {
  static float headingAngle; // Use the gyro to drive straight

  if (junctionIDState == JunctionIDState::DONE) {
    // Actions: N/A
    // State Exit Condition: Always switch to init state (the DONE state is just a marker to
    //                       tell the outside FSM that the turn completed)
    junctionIDState = JunctionIDState::INIT;
  }

  if (junctionIDState == JunctionIDState::INIT) {
    // Actions + State Exit Conditions: Based on the last line sensor reading
    firstLineReading = latestLineReading;

    if (latestLineReading == LineReading::EMPTY) {
      // Empty line readings are always dead ends on init
      identifiedJunction = Junction::DEAD_END;
      junctionIDState = JunctionIDState::DONE;
      playToneFor(identifiedJunction);
    } else {
      // Center the robot's rotation point on the middle of the junction. There,
      // we'll take the final measurement to disambiguate the junction type
      leftMotorController.reset();
      rightMotorController.reset();

      headingAngle = gyro.getAngle();

      leftMotorController.setMaxPosition(ROBOT_HEIGHT_INCHES);
      rightMotorController.setMaxPosition(ROBOT_HEIGHT_INCHES);
      
      junctionIDState = JunctionIDState::CENTERING_ON_JUNCTION;
    }
  }

  if (junctionIDState == JunctionIDState::CENTERING_ON_JUNCTION) {
    // Actions: Drive both motors straight, correcting for slight errors in heading
    double angularAdjustment = (gyro.getAngle() - headingAngle) * ANGLE_ADJUSTMENT_FACTOR;
    double leftSpeed = BASE_SPEED + angularAdjustment;
    double rightSpeed = BASE_SPEED - angularAdjustment;
    updateMotorSpeeds(leftSpeed, rightSpeed);

    // State Exit Conditions: Exit if both motors traveled the desired distance
    if (leftMotorController.reachedMaxPosition() && rightMotorController.reachedMaxPosition()) {
      leftMotor.stop();
      rightMotor.stop();

      junctionIDState = JunctionIDState::DONE;
      identifiedJunction = determineJunction(firstLineReading, latestLineReading);
      playToneFor(identifiedJunction);
    }
  }
}

void turningActions() {
  static PIDController turnController(TURN_CONSTANTS, -MAX_TURN_SPEED, MAX_TURN_SPEED, 
    DEGREE_THRESHOLD);

  if (turnState == TurnState::DONE) {
    // Actions: N/A
    // State Exit Condition: Always switch to init state (the DONE state is just a marker to
    //                       tell the outside FSM that the turn completed)
    turnState = TurnState::INIT;
  }
  
  if (turnState == TurnState::INIT) {
    // Actions: Set target setpoint
    turnController.Reset();
    leftMotorController.reset();
    rightMotorController.reset();
    turnController.SetSetpoint(gyro.getAngle() + pickTurnAngle());

    // State Exit Condition: after INIT, we want to start TURNING immediately
    turnState = TurnState::TURNING;
  }

  if (turnState == TurnState::TURNING) {
    // Actions: Update L and R motor speeds using PID computation
    double motorSpinSpeed = turnController.Compute(gyro.getAngle());
    updateMotorSpeeds(-motorSpinSpeed, motorSpinSpeed);
    
    // State Exit Condition: Only complete once we reached the target setpoint
    if (turnController.ReachedSetpoint()) {
      leftMotor.stop();
      rightMotor.stop();
      turnState = TurnState::DONE;
    }
  }
}

void finishedActions() {
  // Do nothing when we finished the maze :)
}

/**
 * Determine which junction the robot encountered based on two line sensor readings.
 */
Junction determineJunction(LineReading firstReading, LineReading lastReading) {
  switch(firstReading) {
    case LineReading::FULL:
      return (lastReading == LineReading::EMPTY) ? Junction::T : Junction::PLUS;

    case LineReading::LEFT:
      return (lastReading == LineReading::EMPTY) ? Junction::LEFT : Junction::LEFT_T;

    case LineReading::RIGHT:
      return (lastReading == LineReading::EMPTY) ? Junction::RIGHT : Junction::RIGHT_T;

    // This function usually is never called with the following first readings
    // because the last reading has no influence on their behavior
    case LineReading::CENTERED:
      return Junction::LINE;

    case LineReading::EMPTY:
      return Junction::DEAD_END;

    default:
      handleFatalError("Invalid line reading found while determining the junction");
      return Junction::DEAD_END;
  }
}

/**
 * Implements maze-solving logic by determining how much to turn based on
 * the current program state.
 */
int pickTurnAngle() {
  switch (identifiedJunction)
  {
    case Junction::LEFT:
      return 90;

    case Junction::PLUS:
    case Junction::RIGHT:
    case Junction::RIGHT_T:
    case Junction::T:
      return -90;

    case Junction::LEFT_T:
    case Junction::LINE:
      return 0;

    case Junction::DEAD_END:
      return 180;

    default: 
      handleFatalError("Invalid junction type while picking the next turn angle");
      return 0;
  }
}

/**
   Plays a tone associated with a particular junction
*/
void playToneFor(Junction junction, unsigned int duration) {
  Songs::playSound(getToneFor(junction), duration);
}

/**
   Maps junctions to tones
*/
int getToneFor(Junction junction) {
  switch (junction) {
    case Junction::DEAD_END: return NOTE_C6;
    case Junction::RIGHT: return NOTE_E6;
    case Junction::RIGHT_T: return NOTE_G6;
    case Junction::LEFT: return NOTE_C7;
    case Junction::LEFT_T: return NOTE_E7;
    case Junction::T: return NOTE_G7;
    case Junction::PLUS: return NOTE_C8;
    case Junction::LINE: return NOTE_E8;
    default: return NOTE_G8;
  }
}

/**
 * Drives the left and right motors at the desired velocities
 */
void updateMotorSpeeds(double leftSpeed, double rightSpeed) {
  leftMotorController.setTargetVelocity(leftSpeed);
  rightMotorController.setTargetVelocity(rightSpeed);

  leftMotorController.update();
  rightMotorController.update();
}

/**
 * Print out info about the robot status
 */
void printStatus() {
  static int lastPrintTimeMs = millis();
  static std::vector<State> prevStates;

  int now = millis();

  if (now - lastPrintTimeMs >= PRINT_DELAY_MS) {
    Serial.print("Prev States: ");

    for (State state : prevStates) {
       Serial.print(stateAsString(state) + " -> ");
    }
    Serial.println();
    
    Serial.println("Last Junction: " + junctionAsString(identifiedJunction));
    Serial.print("Current Position: "); posEstimator.print();
    Serial.print("Line Sensor Vals: "); lineSensor.printAllSensorValues();
    Serial.print("Left Motor Controller: "); leftMotorController.print();
    Serial.print("Right Motor Controller: "); leftMotorController.print();
    Serial.println();

    prevStates.clear();
    lastPrintTimeMs = now;
  } else {
    // Add any new states that we encounter
    if (prevStates.empty() || prevStates.back() != state) {
      prevStates.push_back(state);
    }
  }
}
