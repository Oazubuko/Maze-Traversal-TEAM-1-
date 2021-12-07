#include <PID_v1.h>
#include <Buzzer.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h> //Integrated Code
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"
#include "MotorPositionController.h"
#include "MotorVelocityController.h"
#include "Songs.h"
#include "Stopwatch.h"
#include "Gyro.h"
#include "PositionEstimator.h"

// Forward Declarations
void driveStraightForever(void);
void driveStraight(double inches);
void turnAngle(double degrees);
void driveUntilJunction();
void centerOnJunction();
void bluetooth_init(); //Integrated Code
void update_directions(String directions); //Integrated Code
String FirstMazeRun();
void OptimizedMazeRun();

// create switch characteristic and allow remote device to read and write
BLEService mazeService("0fe79935-cd39-480a-8a44-06b70f36f248");
BLEUnsignedCharCharacteristic flagCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24a", BLERead | BLEWrite | BLENotify);
BLEStringCharacteristic directionsCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24c", BLERead | BLEWrite,100);

// Turn Controller
double sensedAngle, angularSpeed, targetAngle;
PID turnController(&sensedAngle, &angularSpeed, &targetAngle, 0.2, 0, 0.05, DIRECT);

Buzzer buzzer(10);

LineSensor lineSensor(A3, A2);

Motor leftMotor(3, 2, 6, 7);
Motor rightMotor(4, 5, 9, 8);

MotorPositionController leftPosController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS);
MotorPositionController rightPosController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS);

MotorVelocityController leftVelocityController(leftMotor, leftPosController);
MotorVelocityController rightVelocityController(rightMotor, rightPosController);

Gyro gyro;

PositionEstimator posEstimator(leftMotor, rightMotor, gyro);

String directions; //Integrated Code
bool   hasConnected;  //Integrated Code
bool   hasFirstMessage; // Integrated Code
bool   hasFinished; // Integrated Code
bool   asked4directions;  // Integrated Code


void setup() {
  Serial.begin(9600);
  bluetooth_init(); //Integrated Code
  //Request Directions form Jetson
  flagCharacteristic.setValue(-1); //Integrated Code

  directions       = "None";
  hasConnected     = false;
  hasFirstMessage  = false;
  hasFinished      = false;
  asked4directions = false;

  gyro.begin();
  lineSensor.begin();
  
  turnController.SetMode(AUTOMATIC);
  turnController.SetOutputLimits(-MAX_TURN_SPEED, MAX_TURN_SPEED);
  turnController.SetSampleTime(PID_SAMPLE_PERIOD_MS);

  leftVelocityController.reset();
  rightVelocityController.reset();
}

void loop() 
{

  if (hasConnected && hasFirstMessage && !hasFinished)
  {
      Songs::playStarWarsTheme(buzzer);
      if(directions == "None")  
      {        
          directions = FirstMazeRun();
          update_directions(directions);
          hasFinished = true;
          Songs::playStarWarsTheme(buzzer);
      }//End of if(directions == "None") 
      else
      {
          OptimizedMazeRun();
          hasFinished = true;
      }//End of else 
  }//End of if (hasConnected && hasFirstMessage && !hasFinished)
  else if (hasConnected && !asked4directions)
  {

    flagCharacteristic.setValue(2);
    asked4directions = true;
  }//End of else if (hasConnected && !asked4directions)            


  //polls and handles any events on the queue
  BLE.poll();

  //add delay to prevent continuous polling
  delay(50);
   
}
String FirstMazeRun()
{
bool NotFinished = true;
String directions="";

 //while(NotFinished)
 for(int i=0;i<10;i++)
 {
  
  Junction junction = lineSensor.identifyJunction();
  Serial.println(junctionAsString(junction));

  switch (junction) {
    case Junction::DEAD_END:
    buzzer.sound(NOTE_E7, 200);
    turnAngle(180);
    directions+="B";
    break;      
    case Junction::T:
      buzzer.sound(NOTE_C7, 200);
      directions+="";
    case Junction::RIGHT:
      buzzer.sound(NOTE_G7, 200);
      centerOnJunction();
      turnAngle(-90);
      directions+="R";
      break;

    case Junction::LEFT:
      buzzer.sound(NOTE_B7, 200);
      centerOnJunction();

      // If this is a + junction, and not a left junction, 
      // don't bother turning
      if (lineSensor.identifyJunction() == Junction::DEAD_END) {
        turnAngle(90);
      }
      directions+="L";
      break;

    case Junction::LINE:
    default:
      buzzer.sound(NOTE_A7, 200);
      driveUntilJunction();
      directions+="S";
      break;
  }
 } 
 return(directions);
}

void OptimizedMazeRun()
{
  //directions
  int len = directions.length();
  for(int i=0;i<len;i++)
  {
      driveUntilJunction();
      char current=directions.charAt(i);
      Serial.print("current = ");
      Serial.println(current);
  
      switch (current) 
      {
          case 'B':
             buzzer.sound(NOTE_E7, 200);
             turnAngle(180);
          break;//case 'B':      

          case 'R':
             buzzer.sound(NOTE_G7, 200);
             centerOnJunction();
             turnAngle(-90);
          break;//case 'R':

          case 'L':
            buzzer.sound(NOTE_B7, 200);
            centerOnJunction();

            // If this is a + junction, and not a left junction, 
            // don't bother turning
            if (lineSensor.identifyJunction() == Junction::DEAD_END) 
            {
               turnAngle(90);
            }//End of if (lineSensor.identifyJunction() == Junction::DEAD_END)
      
          break;//case 'L':

          case 'S':
          default:
            buzzer.sound(NOTE_A7, 200);
            driveUntilJunction();
          break;
      }//End of switch (junction)

  }//End of for(int i=0;i<len;i++)
  driveUntilJunction();
}//void OptimizedMazeRun()

/**
 * Turn the robot to a specified angle in degrees (positive = counterclockwise)
 */
void turnAngle(double degrees) {
  Serial.println("Turning to " + String(degrees) + " degrees...");
  rightMotor.stop();
  leftMotor.stop();

  leftVelocityController.reset();
  rightVelocityController.reset();
  
  sensedAngle = gyro.getAngle();
  targetAngle = sensedAngle + degrees;

  Stopwatch setpointTimer;

  // Keep running until we've settled on an angle (so we don't stop
  // as soon as we get a single target measurement)
  while (setpointTimer.getElapsedTime() < SETTLING_TIME) {
    float dt = setpointTimer.lap();
    sensedAngle = gyro.getAngle();

    turnController.Compute();
    leftVelocityController.setTargetVelocity(-angularSpeed);
    rightVelocityController.setTargetVelocity(angularSpeed);

    leftVelocityController.update();
    rightVelocityController.update();

    posEstimator.update();

    if (fabs(sensedAngle - targetAngle) >= DEGREE_THRESHOLD) {
      setpointTimer.zeroOut();
    }

    Serial.println("Moving " + String(sensedAngle) + " -> " + String(targetAngle) + " @ " + String(angularSpeed));
    Serial.println("Time at Target: " + String(setpointTimer.getElapsedTime()));
    Serial.println("Angle Error (degrees): " + String(targetAngle - sensedAngle));

    delay(PID_SAMPLE_PERIOD_MS);
  }

  rightMotor.stop();
  leftMotor.stop();
}

void driveStraightForever() {
  Stopwatch angleStopwatch;
  float startAngle = gyro.getAngle();
  
  leftVelocityController.reset();
  rightVelocityController.reset();

  while (true) {
    double kp = 0.25;
    double angularAdjustment = (gyro.getAngle() - startAngle) * kp;

    leftVelocityController.setTargetVelocity(BASE_SPEED + angularAdjustment);
    rightVelocityController.setTargetVelocity(BASE_SPEED - angularAdjustment);

    leftVelocityController.update();
    rightVelocityController.update();

    posEstimator.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }
}

void driveStraight(double inches) {
  Serial.println("Drive: " + String(inches) + "in.");

  leftVelocityController.reset();
  rightVelocityController.reset();

  leftVelocityController.setTargetVelocity(BASE_SPEED);
  rightVelocityController.setTargetVelocity(BASE_SPEED);

  
  while (!leftPosController.reachedSetpoint() || !rightPosController.reachedSetpoint()) {
    if (leftPosController.getTargetPosition() <= inches) {
      leftVelocityController.update();
    } else {
      leftPosController.setTargetPosition(inches);
      leftPosController.update();
    }

    if (rightPosController.getTargetPosition() <= inches) {
      rightVelocityController.update();
    } else {
      rightPosController.setTargetPosition(inches);
      rightPosController.update();
    }

    posEstimator.update();

    delay(PID_SAMPLE_PERIOD_MS);
  }

  leftMotor.stop();
  rightMotor.stop();
}

void centerOnJunction() {
  Serial.println("Centering on junction");
  driveStraight(ROBOT_HEIGHT_INCHES);
}

void driveUntilJunction() {
  Serial.println("Driving to Junction");

  leftVelocityController.reset();
  rightVelocityController.reset();
  
  while (lineSensor.identifyJunction() == Junction::LINE) {
    // Determine angular adjustment
    // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
    float skew = lineSensor.getSkew();
    leftVelocityController.setTargetVelocity(BASE_SPEED - SKEW_ADJUSTMENT_FACTOR * skew);
    rightVelocityController.setTargetVelocity(BASE_SPEED + SKEW_ADJUSTMENT_FACTOR * skew);

    leftVelocityController.update();
    rightVelocityController.update();

    posEstimator.update();

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
  hasConnected     = false;
  hasFirstMessage  = false;
  asked4directions = false;
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}
