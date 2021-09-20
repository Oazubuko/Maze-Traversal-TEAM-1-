#include <math.h>
#include <Arduino_LSM9DS1.h>
#include "Motor.h"
#include "Constants.h"
#include "LineSensor.h"

// PID Constansts
constexpr PIDConstants distanceConstants = { 0.25, 0, 0 };
constexpr PIDConstants velocityConstants = { 0.25, 0, 0 };
constexpr PIDConstants turningConstants = { 0.25, 0, 0 };

constexpr float DISTANCE_THRESHOLD_INCHES = 0.8;

// Gyro Constants
constexpr float GYRO_BIAS = 0.74; // Degrees / sec

// Testing Constants
int inchesToDrive = 5;

Motor leftMotor(3, 2, A0, 6, 7);
Motor rightMotor(4, 5, A1, 9, 8);

LineSensor lineSensor(A3, A2);

/**
 * Return angular speed in degrees / sec
 */
float getAngularSpeed() {
  if (!IMU.gyroscopeAvailable()) {
    return GYRO_BIAS;
  }
  
  float rotationX, rotationY, rotationZ;
  IMU.readGyroscope(rotationX, rotationY, rotationZ);

  return rotationZ;
}

/**
 * Drives the robot for the specified distance in centimeters
 */
void driveInches(float distanceInInches) {
  // Zero out the encoder values
  leftMotor.resetSensors();
  rightMotor.resetSensors();
  
  float leftError = distanceInInches;
  float rightError = distanceInInches;
  float leftErrorIntegral = 0;
  float rightErrorIntegral = 0;
  
  float angularError = 0;
  
  float dt = 50;
  int prevMicros = micros();
  
  while(fabs(leftError) > DISTANCE_THRESHOLD_INCHES || fabs(rightError) > DISTANCE_THRESHOLD_INCHES) {
    int nowMicros = micros();
    dt = (nowMicros - prevMicros) / 1e6;
    prevMicros = nowMicros;

    angularError += (getAngularSpeed() - GYRO_BIAS) * dt;
    float angleAdjustment = angularError * turningConstants.kp;
    
    leftError = distanceInInches - rightMotor.getInchesDriven();
    rightError = distanceInInches - rightMotor.getInchesDriven();
    rightErrorIntegral += rightError * dt;
    leftErrorIntegral += leftError * dt;

    float leftSpeed = leftError * distanceConstants.kp 
      + leftErrorIntegral * distanceConstants.ki 
      + angleAdjustment;
      
    float rightSpeed = rightError * distanceConstants.kp 
      + rightErrorIntegral * distanceConstants.ki
      - angleAdjustment;

    leftMotor.driveAtSpeed(leftSpeed);
    rightMotor.driveAtSpeed(rightSpeed);

    Serial.print("Left Error (in): ");
    Serial.print(leftError);
    Serial.print("\tIntegral: ");
    Serial.print(leftErrorIntegral);
    Serial.print("\tSpeed: ");
    Serial.println(leftSpeed);

    Serial.print("Right Error (in): ");
    Serial.print(rightError);
    Serial.print("\tIntegral: ");
    Serial.print(rightErrorIntegral);
    Serial.print("\tSpeed: ");
    Serial.println(rightSpeed);
    
    Serial.print("Angular Error: ");
    Serial.println(rightError);

    Serial.println();
    
    delay(20);
  }

  leftMotor.stop();
  rightMotor.stop();
}

void setup() {
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  lineSensor.begin();
  leftMotor.begin();
  rightMotor.begin();
}

void loop() {
  constexpr float baseSpeed = 0.135;
  constexpr float maxSpeedAdjustment = 0.08;
  float leftSpeed, rightSpeed;
  
  float skew = lineSensor.getSkew();

  if (lineSensor.cantSeeAnyTape()) {
    // Drive Backwards
    Serial.println("Driving Backwards");
    leftSpeed = -baseSpeed * 1.25;
    rightSpeed = -baseSpeed * 1.25;
  } else {
    // Determine angular adjustment
    // Positive skew -> robot is tilted right -> need to turn left -> rightMotor high and leftMotor low
    leftSpeed = baseSpeed - maxSpeedAdjustment * skew;
    rightSpeed = baseSpeed + maxSpeedAdjustment * skew;
  }

  
  leftMotor.driveAtSpeed(leftSpeed);
  rightMotor.driveAtSpeed(rightSpeed);

  // Only print occasionally
  if (millis() % 1000 <= 2) {
    Serial.print("Skew = ");
    Serial.println(skew);
    Serial.print("Driving w/ speeds:\t");
    Serial.print(leftSpeed);
    Serial.print("\t");
    Serial.println(rightSpeed);
  }
}
