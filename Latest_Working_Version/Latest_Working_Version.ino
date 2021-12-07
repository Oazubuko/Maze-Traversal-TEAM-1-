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
State state = State::AWAITING_INSTRUCTIONS;
bool inLoop;
int loopCount = 0;
Junction identifiedJunction = Junction::LINE;
LineReading firstLineReading = LineReading::LINE;
float headingAngle = 0; // Start angle while centering on junction
float timeStart;
float timeEnd;
//int row[3], col[3];
int currentR=(NUM_MAZE_ROWS/2), currentC=(NUM_MAZE_COLUMNS/2);
int maze[NUM_MAZE_ROWS][NUM_MAZE_COLUMNS]={-1};

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
void initializeMaze(); //Added to Integrate Machi's Code
void printMaze(); //Added to Integrate Machi's Code
bool updateMaze(int unitsTraveled, int angle); //Added to Integrate Machi's Code
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
int pickRightHandedTurnAngle();
int pickLeftHandedTurnAngle();
Junction junctionFromDirections(LineReading latestLineReading);
void driveForward(double distanceInches);

// create switch characteristic and allow remote device to read and write
BLEService mazeService(MAZE_SERVICE_ID); //Integrated Code
BLEUnsignedCharCharacteristic flagCharacteristic(FLAG_CHARACTERISTIC_ID, BLERead | BLEWrite | BLENotify); //Integrated Code
BLEStringCharacteristic directionsCharacteristic(DIRECTIONS_CHARACTERISTIC_ID, BLERead | BLEWrite, 100); //Integrated Code
String directions       = NO_DIRECTIONS;  //Integrated Code
bool   hasConnected     = false;          //Integrated Code
bool   hasFirstMessage  = false;          // Integrated Code
bool   hasFinished      = false;          // Integrated Code
bool   asked4directions = false;          // Integrated Code
int current_position = 0;                 // Integrated Code

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

  initializeMaze();
  maze[currentR][currentC] =gyro.getCardinalDirectionAngle();
}//End of setup()

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
          directions = "";
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

      if ( (lineReading != LineReading::LINE)&&(current_position<0) ) 
      {
         //
         enterIdentifyingJunctionState(lineReading);
         
         //Add Code to Update Maze
         //calculate delta t, the elapsed time
         timeEnd = micros()*(1e-6);
         float deltaT = timeEnd-timeStart;
         //calculate the number of "units" traveled
         //int unitsTraveled = round(deltaT*UNIT_SPEED);
         float unitsFromOffCenterRobot = UNIT_SPEED / 2;
         int unitsTraveled = round(deltaT * UNIT_SPEED + unitsFromOffCenterRobot);
         //determine the direction of travel
         int card = gyro.getCardinalDirectionAngle();
         //update the relavent maze elements based on the direction of travel
         bool loopDetected = updateMaze(unitsTraveled, card);
         if(loopDetected == true)
         {
            inLoop = true;
         }
         //
         //Serial.println("Row and Column");
         //Serial.print(currentR);
         //Serial.print(" ");
         //Serial.println(currentC);
         //Serial.print("Cells ");
         //Serial.println(deltaT);
         //printMaze();        
         //Transition to Next State
         
      }
      else if ( (lineReading != LineReading::LINE)&&(current_position>=0)&&(current_position<directions.length() ) )
      {
        //Transition to FOLLOWING_DIRECTIONS state
        enterFollowingDirectionsState(lineReading);
      }
      else if ( (lineReading != LineReading::LINE)&&(current_position>=0)&&(current_position>=directions.length() ) )
      {
        //Transition to FOLLOWING_DIRECTIONS state
        enterFinishedState();
      }
      break;

    case State::IDENTIFYING_JUNCTION:
      identifyingJunctionActions(lineReading);

      if (identifiedJunction != Junction::UNKNOWN) {
        if (identifiedJunction == Junction::END_OF_MAZE) {
          enterTransmittingDirectionsState();
        } else if (identifiedJunction == Junction::LINE) {
          enterFollowingLineState();
        } 
        else 
        {
           enterTurningState();          
        }//End of else
      }//End of if (identifiedJunction != Junction::UNKNOWN)
      break;

    case State::TURNING:
      turningActions();

      if (turnController.ReachedSetpoint()) {
        turnController.Print();
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

//Set all elements of the current Maze data to the same initial value
void initializeMaze() //Added to Integrate Machi's Code
{
    for(int j = 0; j < NUM_MAZE_ROWS; j++)
    {
        for(int i = 0; i < NUM_MAZE_COLUMNS; i++)
        {
            maze[j][i]=-1;
        }//End of inner for loop
    }//End of outer for loop

}//End of void initializeMaze()

//Prints the current Maze data to the Serial Line
void printMaze() //Added to Integrate Machi's Code
{
    Serial.println("maze");    
    for(int j = 0; j < NUM_MAZE_ROWS; j++)
    {
        Serial.print("row: ");
        for(int i = 0; i < NUM_MAZE_COLUMNS; i++)
        {
            Serial.print(maze[j][i]);
        }//End of inner for loop
        Serial.println(" ");
    }//End of outer for loop

}//End of void printMazeData()

//Updates the Maze Data after following a Line
bool updateMaze(int unitsTraveled, int angle) //Added to Integrate Machi's Code
{
    bool loopDetected = false;
    for(int i = 0; i < unitsTraveled; i++)
    {
         //Move through maze according to direction of Travel
         if(angle == 90)
         {
             //update maze from [currentR][currentC] to [currentR][currentC-unitsTraveled]
             //currentC=currentC-unitsTraveled;
             currentC--;
         }
         else if( (angle == 270)||(angle == -90) )
         {
             //currentC=currentC+unitsTraveled;
             currentC++;
         }
         else if(abs(angle) == 180)
         {
             //currentR=currentR-unitsTraveled;
             currentR++;
         }
         else if(angle == 0)
         {
             //currentR=currentR+unitsTraveled;
             currentR--;
         }
         
         //Save angle of entry in new location if not previously visited
         //else previously visited check for looping
         if(maze[currentR][currentC] == -1)//location has not been previously visited
         {
             maze[currentR][currentC]=angle;
         }
         else if(maze[currentR][currentC]==angle)//location has been previously entered from the same direction
         {
            loopDetected=true;
         }
         else//location has been previously entered from a different direction
         {
             //Do nothing by design
             //Do Not add code here
         }
         
     }//End of for(int i = 0; i < unitsTraveled; i++)
     return loopDetected;
}//End of bool updateMazeData()

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
    Serial.println("Inside awaiting instructions"); // TODO: removed
  }
}//End of void awaitingInstructionsActions()

/**
   Line Following State
*/
void enterFollowingLineState() 
{
  state = State::FOLLOWING_LINE;

  leftMotorController.reset();
  rightMotorController.reset();

  timeStart = micros()*(1e-6);//Should be Last Line in this function for most accurate results
}//End of void enterFollowingLineState()

void followingLineActions() {
  // Determine angular adjustment using the line sensor's 'skew' measurement
  // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
  float skew = lineSensor.getSkew2();
  //float skew1 = lineSensor.getSkew();

  float leftSpeed = ID_JUNCTION_SPEED - SKEW_ADJUSTMENT_FACTOR * skew;
  float rightSpeed = ID_JUNCTION_SPEED + SKEW_ADJUSTMENT_FACTOR * skew;
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
   Right Handed Turning State
*/
void enterTurningState() {
  state = State::TURNING;

  // Set target angle setpoint
  turnController.Reset();
  leftMotorController.reset();
  rightMotorController.reset();
  float targetAngle=0.0;
  if(inLoop)
  {
      targetAngle = gyro.getAngle() + pickLeftHandedTurnAngle();
  }
  else
  {
      targetAngle = gyro.getAngle() + pickRightHandedTurnAngle();
  }
  turnController.SetSetpoint(targetAngle);
}//End of void enterRightHandedTurningState()

void turningActions() {
  // Actions: Update L and R motor speeds using PID computation
  double motorSpinSpeed = turnController.Compute(gyro.getAngle());
  updateMotorSpeeds(-motorSpinSpeed, motorSpinSpeed);

  if (turnController.ReachedSetpoint()) {
    gyro.alignWithCardinalDirection();
  }
}//End of void rightHandedTurningActions()

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
int pickRightHandedTurnAngle() 
{
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
      directions += "S";
    case Junction::LINE:
      return 0;

    case Junction::DEAD_END:
      directions += "B";
      return 180;

    default:
      handleFatalError("Invalid junction type while picking the next turn angle");
      return 0;
  }
}//End of int pickTurnAngle()

/**
   Implements maze-solving logic by determining how much to turn based on
   the current program state.
*/
int pickLeftHandedTurnAngle() {
  switch (identifiedJunction)
  {
    case Junction::PLUS:
    case Junction::T:
    case Junction::LEFT_T:
      inLoop=false;
    case Junction::LEFT:
      directions += "L";
      return 90;


    case Junction::RIGHT:
      directions += "R";
      return -90;

    case Junction::RIGHT_T:
      directions += "S";
      inLoop=false;
    case Junction::LINE:
      return 0;

    case Junction::DEAD_END:
      directions += "B";
      return 180;

    default:
      handleFatalError("Invalid junction type while picking the next turn angle");
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
  Serial.print(directions);
  Serial.print("current = ");
  Serial.println(current);
  
  switch (current) 
  {
     case 'B':
        buzzer.sound(NOTE_E7, 200);
        return Junction::DEAD_END;
     break;//case 'B':      

     case 'R':
        buzzer.sound(NOTE_G7, 200);
        return Junction::RIGHT;
      break;//case 'R':

      case 'L':
         buzzer.sound(NOTE_B7, 200);
         return Junction::LEFT;
      break;//case 'L':

      case 'S':
         buzzer.sound(NOTE_A7, 200);
         return Junction::LINE;
      break;

      default:
         handleFatalError("Invalid junction type while picking the next turn angle");
         return Junction::UNKNOWN;
      break;
  }//End of switch (current)
  
}//End of int junctionFromDirections()


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
