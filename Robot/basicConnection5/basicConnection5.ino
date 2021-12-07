#include <PID_v1.h> //Integrated Code
#include <Buzzer.h> //Integrated Code
#include <Arduino_LSM9DS1.h> //Integrated Code
#include <ArduinoBLE.h>
#include "Motor.h" //Integrated Code
#include "Constants.h" //Integrated Code
#include "LineSensor.h" //Integrated Code
#include "MotorPositionController.h" //Integrated Code
#include "MotorVelocityController.h" //Integrated Code
#include "Songs.h" //Integrated Code
#include "Stopwatch.h" //Integrated Code
#include "Gyro.h" //Integrated Code
#include "PositionEstimator.h" //Integrated Code

// Forward Declarations/Function Prototypes
void update_directions(String directions);
void bluetooth_init();

// create switch characteristic and allow remote device to read and write
BLEService mazeService("0fe79935-cd39-480a-8a44-06b70f36f248");
BLEUnsignedCharCharacteristic flagCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24a", BLERead | BLEWrite | BLENotify);
BLEStringCharacteristic directionsCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24c", BLERead | BLEWrite,100);

String directions="LBLLLBSBLLBSLL";
int milliseconds;

// Turn Controller
double sensedAngle, angularSpeed, targetAngle; //Integrated Code
PID turnController(&sensedAngle, &angularSpeed, &targetAngle, 0.2, 0, 0.05, DIRECT); //Integrated Code

Buzzer buzzer(10); //Integrated Code

LineSensor lineSensor(A3, A2); //Integrated Code

Motor leftMotor(3, 2, 6, 7); //Integrated Code
Motor rightMotor(4, 5, 9, 8); //Integrated Code

MotorPositionController leftPosController(leftMotor, LEFT_MOTOR_POSITION_CONSTANTS); //Integrated Code
MotorPositionController rightPosController(rightMotor, RIGHT_MOTOR_POSITION_CONSTANTS); //Integrated Code

MotorVelocityController leftVelocityController(leftMotor, leftPosController); //Integrated Code
MotorVelocityController rightVelocityController(rightMotor, rightPosController); //Integrated Code

Gyro gyro; //Integrated Code

PositionEstimator posEstimator(leftMotor, rightMotor, gyro); //Integrated Code

bool hasConnected=false;

//Integrated Code
void setup() 
{
  hasConnected=false;
  Serial.begin(9600);
  bluetooth_init(); //Integrated Code
 
  flagCharacteristic.setValue(-1); //Integrated Code

  gyro.begin();
  lineSensor.begin();
  
  turnController.SetMode(AUTOMATIC);
  turnController.SetOutputLimits(-MAX_TURN_SPEED, MAX_TURN_SPEED);
  turnController.SetSampleTime(PID_SAMPLE_PERIOD_MS);

  leftVelocityController.reset();
  rightVelocityController.reset();

  Serial.print("End of Setup\n");
}

void loop() 
{
  Serial.print("Before If\n");
  if(!hasConnected)
  {
    Serial.print("Before of First While Loop\n");
    //while(!hasConnected)
    //{
    
    //}//End of while(!hasConnected)
    Serial.print("End of First While Loop\n");
    Songs::playStarWarsTheme(buzzer);
  }//End of if(!hasConnected)
  
  int millis2;

  millis2 = millis();
  //currently updates once every 1000ms=1s
  //30-60 seconds may be better
  if((millis2 - milliseconds) > 60000)
  {
    update_directions(directions);
    milliseconds = millis2;
  }

  //polls and handles any events on the queue
  BLE.poll();

  //add delay to prevent continuous polling
  delay(50);
   
}

void blePeripheralConnectHandler(BLEDevice central) 
{
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  hasConnected=true;
  
}

void blePeripheralDisconnectHandler(BLEDevice central) 
{
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  hasConnected=false;
}

void flagCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) 
{
  unsigned char flag = flagCharacteristic.value();
  Serial.print("flagCharacteristicWritten ");
  Serial.println(flag);
  if(flag == 3)
  {
      flagCharacteristic.setValue(2);
  }
  else if(flag == 4)
  {
   directions = directionsCharacteristic.value();    
  Serial.print("directionsCharacteristicWritten: ");
  Serial.println(directions);
  }
}

void update_directions(String directions)
{
  directionsCharacteristic.writeValue(directions);
  flagCharacteristic.setValue(1);
}

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
  String directions="LBLLLBSBLLBSLL";
  directionsCharacteristic.writeValue(directions);
  BLE.advertise();
  Serial.println("Waiting for connection");
}
