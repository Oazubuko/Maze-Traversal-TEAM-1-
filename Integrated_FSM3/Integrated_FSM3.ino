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

// Global state / data
State state = State::FOLLOWING_LINE;
int loopCount = 0;
Junction identifiedJunction = Junction::LINE;
LineReading firstLineReading = LineReading::LINE;
float headingAngle = 0; // Start angle while centering on junction

// Maze state
float time0;
float time1;
int r = MAZE_ROWS / 2;
int c = MAZE_COLS / 2;
int maze[MAZE_ROWS][MAZE_COLS] = {0};
int cells;
int mflag = 0;

// Global objects
LineSensor lineSensor(A3, A2);
Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);
MotorController leftMotorController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorController rightMotorController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);
PIDController turnController(TURN_CONSTANTS, -MAX_TURN_SPEED, MAX_TURN_SPEED, DEGREE_THRESHOLD);
Gyro gyro;
PositionEstimator posEstimator(leftMotor, rightMotor, gyro);
Stopwatch fsmTimer;

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
void turnToAngle(double degrees);
void turnByAngle(double angleIncrement);
void handleFatalError(String errorMessage);
void updateLineSensorHistory();
int pickTurnAngle();
Junction junctionFromDirections(LineReading latestLineReading);
void driveForward(double distanceInches);
void printMaze();
void updateMaze();

// create switch characteristic and allow remote device to read and write
BLEService mazeService(MAZE_SERVICE_ID); //Integrated Code
BLEUnsignedCharCharacteristic flagCharacteristic(FLAG_CHARACTERISTIC_ID, BLERead | BLEWrite | BLENotify); //Integrated Code
BLEStringCharacteristic directionsCharacteristic(DIRECTIONS_CHARACTERISTIC_ID, BLERead | BLEWrite, 100); //Integrated Code
String directions(NO_DIRECTIONS);         //Integrated Code
bool   hasConnected     = false;          //Integrated Code
bool   hasFirstMessage  = false;          // Integrated Code
bool   hasFinished      = false;          // Integrated Code
bool   asked4directions = false;          // Integrated Code
int current_position = -1;                // Integrated Code

void setup() {
  Serial.begin(9600);

  Songs::playStarWarsTheme();

  bluetooth_init(); //Integrated Code
  //Request Directions form Jetson
  //flagCharacteristic.setValue(-1); //Integrated Code

  gyro.begin();
  lineSensor.begin();

  leftMotorController.reset();
  rightMotorController.reset();

  fsmTimer.zeroOut();
}

/**
   The loop is organized as a Finite State Machine. During each loop, we:
   1. Read sensor values
   2. Transition to a new state based on the sensor readings
   3. Output the appropriate signals based on our current state + inputs
*/
void loop() {
  // Read sensors
  LineReading lineReading = lineSensor.getReading();
  gyro.update();
  posEstimator.update();

  // Finite State Machine
  switch (state) {
    case State::AWAITING_INSTRUCTIONS:
      awaitingInstructionsActions();

      if (hasConnected && hasFirstMessage)
      {
        // TODO: remove
        Songs::playSound(NOTE_C4);

        if (directions == NO_DIRECTIONS) {
          Songs::playMarioTheme();
          directions = ""; // The directions will be filled in as we traverse the maze
          current_position = -1;
          enterFollowingLineState();
        } else {
          //directions available
          //state = State::OPTIMIZED_MAZE_RUN; // TODO: add entrance function
          current_position = 0;//needed to ensure the correct startpoint when reading directions
          enterFollowingLineState();
        }
      }
      break;

    case State::FOLLOWING_LINE:
      followingLineActions();

      if (lineReading != LineReading::LINE) {
        if (current_position < 0) {
          // There are no directions to follow
          enterIdentifyingJunctionState(lineReading);
        } else if (current_position >= 0 && current_position < directions.length()) {
          // There are still directions to follow
          enterFollowingDirectionsState(lineReading);
        } else {
          // Received last direction -- at the end of the maze!
          enterFinishedState();
        }
      }
      break;

    case State::IDENTIFYING_JUNCTION:
      identifyingJunctionActions(lineReading);

      if (identifiedJunction != Junction::UNKNOWN) {
        if (identifiedJunction == Junction::END_OF_MAZE) {
          enterTransmittingDirectionsState();
        } else if (identifiedJunction == Junction::LINE) {
          enterFollowingLineState();
        } else {
          updateMaze();          
          enterTurningState();
        }
      }
      break;

    case State::TURNING:
      turningActions();

      if (turnController.ReachedSetpoint()) {
        enterFollowingLineState();
      }
      break;

    case State::FOLLOWING_DIRECTIONS:
      followingDirectionsActions();
      if (identifiedJunction != Junction::UNKNOWN) 
      {
        if (identifiedJunction == Junction::END_OF_MAZE) {
          enterFinishedState();
        } else if (identifiedJunction == Junction::LINE) {
          enterFollowingLineState();
        } else {
          current_position++;
          enterTurningState();
        }
      }
      
      break;

    //case State::OPTIMIZED_MAZE_RUN:
      //optimizedMazeRunActions();
      //break;

    case State::TRANSMITTING_DIRECTIONS:
      transmittingDirectionsActions();
      
      //after confirmed directions transmitted successfully, enterFinishedState();
      if(flagCharacteristic.value() == 3)
      {
        Songs::playJingleBells();
        enterFinishedState();
      }
      break;

    case State::FINISHED:
      finishedActions();
      break;
      
    default:
      handleFatalError("Illegal state in next state logic check");
      break;
  }

  loopCount++;
  printStatus();

  delay(PID_SAMPLE_PERIOD_MS);
}

/**
   Awaiting Instructions State
*/
void awaitingInstructionsActions()
{
  //polls and handles any events on the queue
  BLE.poll();

  if (hasConnected && !asked4directions) {
    flagCharacteristic.setValue(2);
    asked4directions = true;
  }
}//End of void awaitingInstructionsActions()

/**
   Line Following State
*/
void enterFollowingLineState() {
  time0=micros()*(1e-6);
  state = State::FOLLOWING_LINE;

  leftMotorController.reset();
  rightMotorController.reset();
}

void followingLineActions() {
  // Determine angular adjustment using the line sensor's 'skew' measurement
  // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
  float skew = lineSensor.getSkew2();
  float skew1 = lineSensor.getSkew();

  double leftSpeed = ID_JUNCTION_SPEED - SKEW_ADJUSTMENT_FACTOR * skew;
  double rightSpeed = ID_JUNCTION_SPEED + SKEW_ADJUSTMENT_FACTOR * skew;
  updateMotorSpeeds(leftSpeed, rightSpeed);
}

/**
   Identify Junction State
*/
void enterIdentifyingJunctionState(LineReading latestLineReading) {
  state = State::IDENTIFYING_JUNCTION;

  leftMotorController.reset();
  rightMotorController.reset();

  firstLineReading = latestLineReading;
  identifiedJunction = Junction::UNKNOWN;

  switch (firstLineReading) {
    // Empty line readings are always dead ends
    case LineReading::EMPTY:
      identifiedJunction = Junction::DEAD_END;
      playToneFor(identifiedJunction);
      break;

    case LineReading::END_OF_MAZE:
      identifiedJunction = Junction::END_OF_MAZE;
      playToneFor(identifiedJunction);
      break;

    case LineReading::LINE:
      Serial.println("Anakin, you were supposed to follow the lines, not identify them!");
      identifiedJunction = Junction::LINE;
      playToneFor(identifiedJunction);
      break;

    // Unknown junctions are tricky to deal with--let's just play an error
    // tone and treat it like a line
    case LineReading::UNKNOWN:
      Serial.println("Encountered unknown junction--treating it as a line");
      identifiedJunction = Junction::LINE;
      Songs::playErrorSong();
      break;

    // A single one of these readings is ambiguous, so we need to get another check
    // before determining the junction type
    case LineReading::FULL:
    case LineReading::LEFT:
    case LineReading::RIGHT:
      // Center the robot's rotation point on the middle of the junction. There,
      // we'll take the final measurement to disambiguate the junction type
      headingAngle = gyro.getAngle();

      leftMotorController.setMaxPosition(ROBOT_HEIGHT_INCHES);
      rightMotorController.setMaxPosition(ROBOT_HEIGHT_INCHES);
      break;

    default:
      LineSensor::printLineReading(latestLineReading);
      handleFatalError("Found an impossible line reading!!!");
      break;
  }
}

void identifyingJunctionActions(LineReading latestLineReading) {
  // Drive both motors straight, correcting for slight errors in heading
  double angularAdjustment = (gyro.getAngle() - headingAngle) * ANGLE_ADJUSTMENT_FACTOR;
  double leftSpeed = BASE_SPEED + angularAdjustment;
  double rightSpeed = BASE_SPEED - angularAdjustment;
  updateMotorSpeeds(leftSpeed, rightSpeed);

  if (leftMotorController.reachedMaxPosition() && rightMotorController.reachedMaxPosition()) {
    leftMotor.stop();
    rightMotor.stop();

    identifiedJunction = determineJunction(firstLineReading, latestLineReading);
    playToneFor(identifiedJunction);
  }
}

/**
   Turning State
*/
void enterTurningState() {
  state = State::TURNING;

  // Set target angle setpoint
  turnController.Reset();
  leftMotorController.reset();
  rightMotorController.reset();
  float targetAngle = gyro.getAngle() + pickTurnAngle();
  turnController.SetSetpoint(targetAngle);
}

void turningActions() {
  // Actions: Update L and R motor speeds using PID computation
  double motorSpinSpeed = turnController.Compute(gyro.getAngle());
  updateMotorSpeeds(-motorSpinSpeed, motorSpinSpeed);

  if (turnController.ReachedSetpoint()) {
    gyro.alignWithCardinalDirection();
  }
}

/**
   Following Directions State
*/
void enterFollowingDirectionsState(LineReading latestLineReading) {
  state = State::FOLLOWING_DIRECTIONS;

  leftMotorController.reset();
  rightMotorController.reset();

  firstLineReading = latestLineReading;
  identifiedJunction = Junction::UNKNOWN;

  switch (firstLineReading) {
    // Empty line readings are always dead ends
    case LineReading::EMPTY:
      identifiedJunction = Junction::DEAD_END;
      playToneFor(identifiedJunction);
      break;

    case LineReading::END_OF_MAZE:
      identifiedJunction = Junction::END_OF_MAZE;
      playToneFor(identifiedJunction);
      break;

    case LineReading::LINE:
      Serial.println("Anakin, you were supposed to follow the lines, not identify them!");
      identifiedJunction = Junction::LINE;
      playToneFor(identifiedJunction);
      break;

    // Unknown junctions are tricky to deal with--let's just play an error
    // tone and treat it like a line
    case LineReading::UNKNOWN:
      Serial.println("Encountered unknown junction--treating it as a line");
      identifiedJunction = Junction::LINE;
      Songs::playErrorSong();
      break;

    // A single one of these readings is ambiguous, so we need to get another check
    // before determining the junction type
    case LineReading::FULL:
    case LineReading::LEFT:
    case LineReading::RIGHT:
      // Center the robot's rotation point on the middle of the junction. There,
      // we'll take the final measurement to disambiguate the junction type
      headingAngle = gyro.getAngle();

      leftMotorController.setMaxPosition(ROBOT_HEIGHT_INCHES);
      rightMotorController.setMaxPosition(ROBOT_HEIGHT_INCHES);
      break;

    default:
      LineSensor::printLineReading(latestLineReading);
      handleFatalError("Found an impossible line reading!!!");
      break;
  }
}

//currently functionally identical to turningActions
void followingDirectionsActions() {
  // Drive both motors straight, correcting for slight errors in heading
  double angularAdjustment = (gyro.getAngle() - headingAngle) * ANGLE_ADJUSTMENT_FACTOR;
  double leftSpeed = BASE_SPEED + angularAdjustment;
  double rightSpeed = BASE_SPEED - angularAdjustment;
  updateMotorSpeeds(leftSpeed, rightSpeed);

  if (leftMotorController.reachedMaxPosition() && rightMotorController.reachedMaxPosition()) {
    leftMotor.stop();
    rightMotor.stop();

    if( (firstLineReading!=LineReading::LINE)&&(firstLineReading!=LineReading::UNKNOWN) )
    {
        identifiedJunction = junctionFromDirections();
    }
    else
    {
        identifiedJunction = Junction::LINE;
    }
    playToneFor(identifiedJunction);
  }
}


/**
 * Transmitting Directions State
 */
void enterTransmittingDirectionsState() 
{
  state = State::TRANSMITTING_DIRECTIONS;

  leftMotor.stop();
  rightMotor.stop();
  update_directions(directions);
  hasFinished = true;
}

void transmittingDirectionsActions() 
{
   //BLE.poll() needed to trigger transfer of data to Jetson
   BLE.poll();
}

/**
   Finished State
*/
void enterFinishedState() {
  state = State::FINISHED;

  leftMotor.stop();
  rightMotor.stop();
}

void finishedActions() {
}


/**
   Helper Functions
*/

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
    // TODO: remove if not necessary
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
      directions += "L";
      return 90;

    case Junction::PLUS:
    case Junction::RIGHT:
    case Junction::RIGHT_T:
    case Junction::T:
      directions += "R";
      return -90;

    case Junction::LEFT_T:
    case Junction::LINE:
      directions += "S";
      return 0;

    case Junction::DEAD_END:
      directions += "B";
      return 180;

    default:
      handleFatalError("Invalid junction type while picking the next turn angle: " + junctionAsString(identifiedJunction));
      return 0;
  }
}//End of int pickTurnAngle()


/**
   Implements maze-solving logic by determining how much to turn based on
   the current program state.
*/
Junction junctionFromDirections() {
  //current_position = 0;
    //directions
  //int len = directions.length();
  char current=directions.charAt(current_position);
  Serial.println(directions);
  Serial.print("current = ");
  Serial.println(current);
  
  switch (current) 
  {
     case 'B':
        buzzer.sound(NOTE_E7, 200);
        return Junction::DEAD_END;

     case 'R':
        buzzer.sound(NOTE_G7, 200);
        return Junction::RIGHT;

      case 'L':
         buzzer.sound(NOTE_B7, 200);
         return Junction::LEFT;

      case 'S':
         buzzer.sound(NOTE_A7, 200);
         return Junction::LINE;

      default:
         handleFatalError("Invalid direction: " + current);
         return Junction::UNKNOWN;
  }//End of switch (current)
  
}//End of int junctionFromDirections()

/**
 * Updates the robot's current position in the maze. 
 */
void updateMaze() {
    // Compute how many cells we've traveled
    time1 = micros()*(1e-6);
    time1 = time1-time0;
    cells = round(time1);
    
    mflag++;
    
    int cardinalAngle = gyro.getCardinalAngle();
  
    // mflag controls whether we fill in gaps between a junction or not
    if (mflag == 1) {
      cells = 1;
    }
  
    // "Visit" all cells in between the last and the current one
    for (int i = 0; i < cells; i++) {
        // Add 1 to visited cell count
        maze[r][c]++;
  
        if (cardinalAngle == 0) {
          r--;
        } else if (cardinalAngle == 90) {
          c--;
        } else if (cardinalAngle == 180) {
          r++;
        } else if (cardinalAngle == 270) {
          c++;
        } else {
          handleFatalError("ERROR: Invalid Cardinal Angle: " + String(cardinalAngle));
        }
    }
    
    Serial.println("new Row and Column!");
    Serial.print(r);
    Serial.print(" ");
    Serial.println(c);
    Serial.print("Cells added ");
    Serial.println(cells);  
    
    Serial.println("Current Maze: ");
    printMaze();
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
    Serial.print("First Line Reading (of ID state): "); LineSensor::printLineReading(firstLineReading);
    Serial.println("Last Junction: " + junctionAsString(identifiedJunction));
    Serial.print("Current Position: "); posEstimator.print();
    Serial.print("Line Sensor Vals: "); lineSensor.printAllSensorValues();
    Serial.print("Left Motor Controller: "); leftMotorController.print();
    Serial.print("Right Motor Controller: "); rightMotorController.print();
    Serial.println();

    prevStates.clear();
    lastPrintTimeMs = now;
  }
}

/**
   Print the states separated by an arrow (-->), e.g.:
   "TURNING --> IDENTIFYING_JUNCTION --> FOLLOWING_LINE"
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

void printMaze() {
  String columnLabels("  \t");
  for (int col = 0; col < MAZE_COLS; col++) {
    columnLabels += col;
    
    // Add 1 or 2 spaces depending on how whehter the current column is 1 or 2 digits.
    // This functionality will break if the number of columns exceeds 99.
    columnLabels += (col < 10) ? "  " : " ";
  }
  
  String mazeString;
  for (int row = 0; row < MAZE_ROWS; row++) {
    mazeString += row;
    mazeString += '\t';
    
    for (int col = 0; col < MAZE_COLS; col++) {
      mazeString += maze[row][col];
      mazeString += "  "; // 2 spaces
    }
    mazeString += '\n';
  }

  Serial.println(columnLabels);
  Serial.println(mazeString);
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

/**
   Blocking function to turn the robot to a target angle
*/
void turnToAngle(double targetAngle) {
  leftMotorController.reset();
  rightMotorController.reset();
  turnController.Reset();
  turnController.SetSetpoint(targetAngle);

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

/**
   Blocking function to turn the robot by some angular offset (positive angle = counterclockwise)
*/
void turnByAngle(double angleIncrement) {
  float targetAngle = gyro.getAngle() + angleIncrement;
  turnToAngle(targetAngle);
}

/**
   Blocking function to drive the robot straight for given number of inches
*/
void driveForward(double inches) {
  leftMotorController.reset();
  rightMotorController.reset();
  headingAngle = gyro.getAngle();

  leftMotorController.setMaxPosition(inches);
  rightMotorController.setMaxPosition(inches);

  while (!leftMotorController.reachedMaxPosition() && !rightMotorController.reachedMaxPosition()) {
    gyro.update();

    double angularAdjustment = (gyro.getAngle() - headingAngle) * ANGLE_ADJUSTMENT_FACTOR;
    double leftSpeed = ID_JUNCTION_SPEED + angularAdjustment;
    double rightSpeed = ID_JUNCTION_SPEED - angularAdjustment;
    updateMotorSpeeds(leftSpeed, rightSpeed);

    Serial.println(String(leftMotorController.getTargetPosition()) + "\t" + String(leftMotor.getInchesDriven()) + "\t" +
                   String(rightMotorController.getTargetPosition()) + "\t" + String(rightMotor.getInchesDriven()));

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

  if (flag == 4)
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

  BLE.setLocalName(MOUSE_NAME);
  BLE.setAdvertisedService(mazeService);
  mazeService.addCharacteristic(flagCharacteristic);
  mazeService.addCharacteristic(directionsCharacteristic);
  BLE.addService(mazeService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  flagCharacteristic.setEventHandler(BLEWritten, flagCharacteristicWritten);
  flagCharacteristic.setValue(-1);
  directionsCharacteristic.writeValue(NO_DIRECTIONS);
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
