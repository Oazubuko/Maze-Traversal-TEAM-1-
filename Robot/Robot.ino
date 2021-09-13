#include <math.h>
#include "Motor.h"
#include <Arduino_LSM9DS1.h>

const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 9;
const unsigned int M2_ENC_B = 8;

// PID Constansts
constexpr float KP_MOTORS = 0.3;
constexpr float KI_MOTORS = 0.05;
constexpr float KP_TURN = 0;
constexpr float DISTANCE_THRESHOLD_INCHES = 0.8;

// Gyro Constants
constexpr float GYRO_BIAS = 0.74; // Degrees / sec

// Testing Constants
int inchesToDrive = 5;

Motor leftMotor(3, 2, A0, M1_ENC_A, M1_ENC_B, 0.4);
Motor rightMotor(4, 5, A1, M2_ENC_A, M2_ENC_B, 0.4);

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
    float angleAdjustment = angularError * KP_TURN;
    
    leftError = distanceInInches - rightMotor.getInchesDriven();
    rightError = distanceInInches - rightMotor.getInchesDriven();
    rightErrorIntegral += rightError * dt;
    leftErrorIntegral += leftError * dt;

    float leftSpeed = leftError * KP_MOTORS 
      + leftErrorIntegral * KI_MOTORS 
      + angleAdjustment;
      
    float rightSpeed = rightError * KP_MOTORS 
      + rightErrorIntegral * KI_MOTORS
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

void setup() {
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  leftMotor.begin();
  rightMotor.begin();
}

void loop() {
  driveInches(inchesToDrive);
  inchesToDrive += 5;
  delay(3000);
  
}
