#include <vector>
#include <PID_v1.h>
#include <Buzzer.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h> //Integrated Code
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"
#include "MotorController.h"
#include "Songs.h"
#include "Stopwatch.h"
#include "Gyro.h"
#include "PositionEstimator.h"
#include "StateMachine.h"

// Track the most recent line reading and how frequent it is
struct LineHistory {
  LineReading lastReading;
  int occurences;
} lineHistory;

// Turning FSMs
enum class TurnState {INIT, TURNING, DONE};
TurnState turnState = TurnState::INIT;

// Junction identification FSM
enum class JunctionIDState {INIT, CENTERING_ON_JUNCTION, DONE};
JunctionIDState junctionIDState = JunctionIDState::INIT;
Junction identifiedJunction;
LineReading firstLineReading;

// Forward Declarations
void bluetooth_init(); //Integrated Code
void update_directions(String directions); //Integrated Code
void playToneFor(Junction junction, unsigned int duration = 50);
int getToneFor(Junction junction);
int pickTurnDirection();
void printStatus();
void updateMotorSpeeds(double leftSpeed, double rightSpeed);
Junction determineJunction(LineReading firstReading, LineReading lastReading);
void printStates(std::vector<State>& states);
void turnAngle(double degrees);
void handleFatalError(String errorMessage);
void updateLineSensorHistory();

// Global objects
State state = State::AWAITING_INSTRUCTIONS; //state variable
int loopCount = 0;
//
LineSensor lineSensor(A3, A2);
Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);
MotorController leftMotorController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorController rightMotorController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);
PIDController turnController(TURN_CONSTANTS, -MAX_TURN_SPEED, MAX_TURN_SPEED, DEGREE_THRESHOLD);
Gyro gyro;
PositionEstimator posEstimator(leftMotor, rightMotor, gyro);
Stopwatch fsmTimer;

// create switch characteristic and allow remote device to read and write
BLEService mazeService("0fe79935-cd39-480a-8a44-06b70f36f248"); //Integrated Code
BLEUnsignedCharCharacteristic flagCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24a", BLERead | BLEWrite | BLENotify); //Integrated Code
BLEStringCharacteristic directionsCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24c", BLERead | BLEWrite,100); //Integrated Code
String directions; //Integrated Code
bool   hasConnected;  //Integrated Code
bool   hasFirstMessage; // Integrated Code
bool   hasFinished; // Integrated Code
bool   asked4directions;  // Integrated Code

void setup() 
{
  state = State::AWAITING_INSTRUCTIONS; //ensure desired intital state
  Serial.begin(9600);

  bluetooth_init(); //Integrated Code
  //Request Directions form Jetson
  //flagCharacteristic.setValue(-1); //Integrated Code

  directions       = "None"; //Integrated Code
  hasConnected     = false; //Integrated Code
  hasFirstMessage  = false; //Integrated Code
  hasFinished      = false; //Integrated Code
  asked4directions = false; //Integrated Code
  
  Songs::playStarWarsTheme();

  gyro.begin();
  lineSensor.begin();

  leftMotorController.reset();
  rightMotorController.reset();

  fsmTimer.zeroOut();
}//End of void setup()

/**
   The loop is organized as a Finite State Machine. During each loop, we:
   1. Read sensor values
   2. Transition to a new state based on the sensor readings
   3. Output the appropriate signals based on our current state + inputs
*/
void loop() 
{  
  // Read sensors
  //updateLineSensorHistory();
  LineReading lineReading = lineSensor.getReading();
  LineSensor::printLineReading(lineReading);
  gyro.update();
  posEstimator.update();

  // Next State logic
  switch (state) 
  {
    case State::AWAITING_INSTRUCTIONS:
      state = awaitingInstructionsNextState();
      break;
      
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
      
    case State::OPTIMIZED_MAZE_RUN:
      state = optimizedMazeRunNextState();
      break;
      
    case State::TRANSMITTING_DIRECTIONS:
      state = transmittingDirectionsNextState();
      break;
      
    case State::SLEEP:
      state = sleepNextState();
      break;

    default:
      handleFatalError("Illegal state in next state logic check");
      break;
  }//End of switch (state)

  // Output Logic
  switch (state) 
  {
    case State::AWAITING_INSTRUCTIONS:
      awaitingInstructionsActions();
      break;

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
      finishedActions();
      break;

    case State::OPTIMIZED_MAZE_RUN:
      optimizedMazeRunActions();
      break;
      
    case State::TRANSMITTING_DIRECTIONS:
      transmittingDirectionsActions();
      break;
      
    case State::SLEEP:
      sleepActions();
      break;

    default:
      handleFatalError("Illegal state in next state logic check");
      break;
  }//switch (state)

  printStatus();

  loopCount++;

  delay(PID_SAMPLE_PERIOD_MS);
}//End of void loop()

// State transitions
State awaitingInstructionsNextState() 
{
  State nextState;
  if (hasConnected && hasFirstMessage)
  {
     if(directions == "None")
     {
        nextState=State::FOLLOWING_LINE;
     }
     else//directions available
     {
        nextState=State::OPTIMIZED_MAZE_RUN;
     }
  }
  else
  {
      nextState=State::AWAITING_INSTRUCTIONS;
  }
  //
  return nextState;
}//State awaitingInstructionsNextState()

State followingLineNextState(LineReading lineReading) {
  if (lineReading != LineReading::LINE) return State::IDENTIFYING_JUNCTION;

  return State::FOLLOWING_LINE;
}

State identifyingJunctionNextState() {
  if (junctionIDState == JunctionIDState::DONE) {
    if (identifiedJunction == Junction::END_OF_MAZE) {
      return State::FINISHED;
    } else if (identifiedJunction == Junction::LINE) {
      return State::FOLLOWING_LINE;
    } else {
      return State::TURNING;
    }
  }

  return State::IDENTIFYING_JUNCTION;
}

State turningNextState() {
  if (turnState == TurnState::DONE) return State::FOLLOWING_LINE;

  return State::TURNING;
}

State finishedNextState() {
  return State::FINISHED;
}

State optimizedMazeRunNextState() 
{
  //if (lineReading != LineReading::LINE) return State::IDENTIFYING_JUNCTION;

  //return State::FOLLOWING_LINE;
}

State transmittingDirectionsNextState() 
{
  //if (lineReading != LineReading::LINE) return State::IDENTIFYING_JUNCTION;

  //return State::FOLLOWING_LINE;
}

State sleepNextState() 
{
  //if (lineReading != LineReading::LINE) return State::IDENTIFYING_JUNCTION;

  //return State::FOLLOWING_LINE;
}


// State actions
void awaitingInstructionsActions() 
{
  //polls and handles any events on the queue
  BLE.poll();

  if (hasConnected && !asked4directions)
  {
    flagCharacteristic.setValue(2);
    asked4directions = true;
  }//End of else if (hasConnected && !asked4directions)

  //add delay to prevent continuous polling
  delay(50);
}//End of void awaitingInstructionsActions()

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
    leftMotorController.reset();
    rightMotorController.reset();

    firstLineReading = latestLineReading;

    switch (firstLineReading) {
      // Empty line readings are always dead ends
      case LineReading::EMPTY:
        identifiedJunction = Junction::DEAD_END;
        junctionIDState = JunctionIDState::DONE;
        playToneFor(identifiedJunction);
        break;

      case LineReading::LINE:
        Serial.println("Anakin, you were supposed to follow the lines, not identify them!");
        identifiedJunction = Junction::LINE;
        junctionIDState = JunctionIDState::DONE;
        playToneFor(identifiedJunction);
        break;

      // Unknown junctions are tricky to deal with--let's just play an error
      // tone and treat it like a line
      case LineReading::UNKNOWN:
        Serial.println("Encountered unknown junction--treating it as a line");
        identifiedJunction = Junction::LINE;
        junctionIDState = JunctionIDState::DONE;
        Songs::playErrorSong();
        break;

      // A single one of these readings is ambiguous, so we need to get another check
      // before determining the junction type
      case LineReading::FULL:
      case LineReading::LEFT:
      case LineReading::RIGHT:
      case LineReading::END_OF_MAZE:
        // Center the robot's rotation point on the middle of the junction. There,
        // we'll take the final measurement to disambiguate the junction type
        headingAngle = gyro.getAngle();

        leftMotorController.setMaxPosition(ROBOT_HEIGHT_INCHES);
        rightMotorController.setMaxPosition(ROBOT_HEIGHT_INCHES);

        junctionIDState = JunctionIDState::CENTERING_ON_JUNCTION;
        break;

      default:
        LineSensor::printLineReading(latestLineReading);
        handleFatalError("Found an impossible line reading!!!");
        break;
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
    Serial.println("Target Angle: " + String(turnController.GetSetpoint()));
    Serial.println("Current Angle: " + String(gyro.getAngle()));
    turnController.Print();
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
  leftMotor.stop();
  rightMotor.stop();
}

void optimizedMazeRunActions() 
{
  
}//End of void optimizedMazeRunActions()

void transmittingDirectionsActions() 
{
  
}//End of void transmittingDirectionsActions()

void sleepActions() 
{
  
}//End of void sleepActions()

/**
   Determine which junction the robot encountered based on two line sensor readings.
*/
Junction determineJunction(LineReading firstReading, LineReading lastReading) {
  switch (firstReading) {
    case LineReading::FULL:
      return (lastReading == LineReading::EMPTY) ? Junction::T : Junction::PLUS;

    case LineReading::LEFT:
      return (lastReading == LineReading::EMPTY) ? Junction::LEFT : Junction::LEFT_T;

    case LineReading::RIGHT:
      return (lastReading == LineReading::EMPTY) ? Junction::RIGHT : Junction::RIGHT_T;

    // This one's tricky--in practice, end of maze readings can occur incorrectly
    // We don't want to stop traversal in that case, so we play an error sound and
    // return a sensible fallback junction
    // TODO: add reversing in the future to reapproach a junction?
    case LineReading::END_OF_MAZE:
      if (lastReading == LineReading::END_OF_MAZE) return Junction::END_OF_MAZE;

      // Ooops, we misidentified the end of the maze!
      // Let's play an error sound and return a dead end. In the worst case, the
      // robot will backtrack to this particular spot.
      Serial.println("Error: incorrectly identified an end-of-maze junction");
      Songs::playErrorSong();
      return Junction::DEAD_END;

    // This function usually is never called with the following first readings
    // because the last reading has no influence on their behavior
    case LineReading::LINE:
      return Junction::LINE;

    case LineReading::EMPTY:
      return Junction::DEAD_END;

    case LineReading::UNKNOWN:
      Songs::playErrorSong();
      Serial.println("Invalid first junction reading: unknown junction!");
      return Junction::LINE; // Best to just treat unknown readings like a line, so the robot drives forward

    default:
      handleFatalError("Invalid line reading found while determining the junction (reached default case)");
      return Junction::DEAD_END;
  }
}

/**
   Implements maze-solving logic by determining how much to turn based on
   the current program state.
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
    case Junction::END_OF_MAZE: return NOTE_F8;
    default: return NOTE_G8;
  }
}

/**
   Drives the left and right motors at the desired velocities
*/
void updateMotorSpeeds(double leftSpeed, double rightSpeed) {
  leftMotorController.setTargetVelocity(leftSpeed);
  rightMotorController.setTargetVelocity(rightSpeed);

  leftMotorController.update();
  rightMotorController.update();
}

/**
   Print out info about the robot status
*/
void printStatus() {
  static int lastPrintTimeMs = millis();
  static std::vector<State> prevStates;

  // Add any new states that we encounter
  if (prevStates.empty() || prevStates.back() != state) {
    prevStates.push_back(state);
  }

  int now = millis();

  if (now - lastPrintTimeMs >= PRINT_DELAY_MS) {
    Serial.print("Previous States: "); printStates(prevStates);
    Serial.println("Average dt: " + String(fsmTimer.getElapsedTime() / loopCount * 1000, 2) + " ms");
    Serial.println("Last Junction: " + junctionAsString(identifiedJunction));
    Serial.print("Current Position: "); posEstimator.print();
    Serial.print("Line Sensor Vals: "); lineSensor.printAllSensorValues();
    Serial.print("Left Motor Controller: "); leftMotorController.print();
    Serial.print("Right Motor Controller: "); leftMotorController.print();
    Serial.println();

    prevStates.clear();
    lastPrintTimeMs = now;
  }
}

/**
 * Print the states separated by an arrow (-->), e.g.:
 * "TURNING --> IDENTIFYING_JUNCTION --> FOLLOWING_LINE"
 */
void printStates(std::vector<State>& states) {
  String stateString;
  
  for (State state : states) {
    if (stateString != "") {
      stateString += " --> ";
    }

    stateString += stateAsString(state);
  }

  Serial.println(stateString);
}

/**
   Enter an error state until the end of time! Spooky.
*/
void handleFatalError(String errorMessage) {
  leftMotor.stop();
  rightMotor.stop();

  Serial.println("Encountered a fatal error: " + errorMessage);
  Songs::playMarioTheme();

  while (true) {
    delay(1000);
  }
}

void turnAngle(double targetAngle) {                                    
  turnController.Reset();
  leftMotorController.reset();
  rightMotorController.reset();
  turnController.SetSetpoint(gyro.getAngle() + targetAngle);
  
  while (!turnController.ReachedSetpoint()) {
    gyro.update();
    double motorSpinSpeed = turnController.Compute(gyro.getAngle());
    updateMotorSpeeds(-motorSpinSpeed, motorSpinSpeed);

    Serial.println(String(turnController.GetSetpoint()) + "\t" + String(gyro.getAngle()));
    
    delay(PID_SAMPLE_PERIOD_MS);
  }
  
  leftMotor.stop();
  rightMotor.stop();
}

//Integrated Code
void flagCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) 
{
  unsigned char flag = flagCharacteristic.value();
  Serial.print("flagCharacteristicWritten ");
  Serial.println(flag);

 if(flag == 4)
  {
   directions = directionsCharacteristic.value();  
   hasFirstMessage = true;  
  Serial.print("directionsCharacteristicWritten: ");
  Serial.println(directions);
  }
}

//Integrated Code
void update_directions(String directions)
{
  directionsCharacteristic.writeValue(directions);
  flagCharacteristic.setValue(1);
}

//Integrated Code
void bluetooth_init()
{
 if (!BLE.begin()) 
 {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Set the connection interval to be as fast as possible (about 40 Hz)
  BLE.setConnectionInterval(0x0006, 0x0050);

  BLE.setLocalName("Ed's Mouse :)");
  BLE.setAdvertisedService(mazeService);
  mazeService.addCharacteristic(flagCharacteristic);
  mazeService.addCharacteristic(directionsCharacteristic);
  BLE.addService(mazeService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  flagCharacteristic.setEventHandler(BLEWritten, flagCharacteristicWritten);
  flagCharacteristic.setValue(-1);
  String directions="None";
  directionsCharacteristic.writeValue(directions);
  BLE.advertise();
  Serial.println("Waiting for connection");
}

//Integrated Code
//Needed for Clean Exit
void blePeripheralConnectHandler(BLEDevice central) 
{
  hasConnected = true;
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

//Integrated Code
//Needed for Clean Exit
void blePeripheralDisconnectHandler(BLEDevice central) 
{
  //BLE.disconnect();
  hasConnected     = false;
  hasFirstMessage  = false;
  asked4directions = false;
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}
