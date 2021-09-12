#include <math.h>
#include "Motor.h"
#include <Encoder.h>
#include <Arduino_LSM9DS1.h>

const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;

// PID Constansts
constexpr float KP_MOTORS = 0.2;
constexpr float KI_MOTORS = 0.1;
constexpr float KP_TURN = 0.03;
constexpr float DISTANCE_THRESHOLD_INCHES = 0.8;

// Calculating Encoder Ticks -> Inches
constexpr float WHEEL_DIAMETER_INCHES = 1.5;
constexpr int TICKS_PER_REV = 360;
constexpr float INCHES_PER_REV = PI * WHEEL_DIAMETER_INCHES;
constexpr int GEAR_RATIO = 30 / 1;
constexpr float ENCODER_TICKS_TO_INCHES = INCHES_PER_REV / TICKS_PER_REV;

// Gyro Constants
constexpr float GYRO_BIAS = 0.74; // Degrees / sec

// Testing Constants
int inchesToDrive = 5;

Motor leftMotor(3, 2, A0, 0.4);
Motor rightMotor(4, 5, A1, 0.4);

Encoder leftEncoder(M1_ENC_A, M1_ENC_B);
Encoder rightEncoder(M2_ENC_A, M2_ENC_B);

/**
 * Drives the robot for the specified distance in centimeters
 */
void driveInches(float distanceInInches) {
  // Zero out the encoder values
  leftEncoder.write(0);
  rightEncoder.write(0);
  
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
    
    leftError = distanceInInches - getDistanceDriven(leftEncoder);
    leftErrorIntegral += leftError * dt;

    // We have to negate the sign here since forward movement of the right motor
    // corresponds to negative right encoder ticks
    rightError = distanceInInches + getDistanceDriven(rightEncoder);
    rightErrorIntegral += rightError * dt;

    angularError += (getAngularSpeed() - GYRO_BIAS) * dt;
    float angleAdjustment = angularError * KP_TURN;

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
 * Transforms encoder ticks to inches
 */
float getDistanceDriven(Encoder encoder) {
  return ENCODER_TICKS_TO_INCHES * encoder.read();
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
