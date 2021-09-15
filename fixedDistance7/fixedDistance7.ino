/*********************************** Test 5******************************/
#include <math.h>
#include "Motor.h"
#include <Arduino_LSM9DS1.h>

#define M1_ENC_A 6 //
#define M1_ENC_B 7 //
#define M2_ENC_A 8 //
#define M2_ENC_B 9 //

#define MAX_PWM  50
#define M1_IN_1  2
#define M1_IN_2  3
#define M2_IN_1  5
#define M2_IN_2  4

// PID Constansts
constexpr float KP_MOTORS = 0.3;
constexpr float KI_MOTORS = 0.05;
constexpr float KP_TURN = 0;
constexpr float DISTANCE_THRESHOLD_INCHES = 0.8;

// Gyro Constants
constexpr float GYRO_BIAS = 0.74; // Degrees / sec

int   pos1      = 0;
int   pos2      = 0;
long  prevT     = 0;
float eprev     = 0;
float eintegral = 0;

int flag=0;

Motor leftMotor(M1_IN_2, M1_IN_1, A0, M1_ENC_A, M1_ENC_B, 0.4);
Motor rightMotor(M2_IN_2, M2_IN_1, A1, M2_ENC_A, M2_ENC_B, 0.4);

void setMotor(int dir, int pwmVal, Motor m);
void readEncoder1();
void readEncoder2();
float getAngularSpeed();

void setup()
{
    Serial.begin(9600);
    //pinMode(M1_ENC_A,INPUT);
    //pinMode(M1_ENC_B,INPUT);
    //pinMode(M2_ENC_A,INPUT);
    //pinMode(M2_ENC_B,INPUT);
    if (!IMU.begin()) 
    {
       Serial.println("Failed to initialize IMU!");
       while (1);
    }

    leftMotor.resetSensors();
    rightMotor.resetSensors();

    leftMotor.begin();
    rightMotor.begin();
    
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A),readEncoder1,RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A),readEncoder2,RISING);
}


void loop()
{
  //if(!flag)
  //{
     // set target position
     //int target = 1200;
     int target = 500;
 
     float kp = 1;     
     float kd = 0.0125;     // 0.0125
     float ki = 0;

     long currT = micros();

     float deltaT = ((float)(currT-prevT)/1.0e6);
     prevT        = currT;

   // error
     int e = pos1 - target;

   //derivative
     float dedt = (e-eprev)/(deltaT);

   //integral
     eintegral = eintegral + e*deltaT;

   //control signal
     float u=kp*e + kd*dedt + ki*eintegral;

     float pwr = fabs(u);
     if(pwr >MAX_PWM)
     {
        pwr = MAX_PWM;
     }

   //motor direction
     int dir = 1;
     if(u< 0)
     {
        dir = -1;
     }

   //signal the motor
     setMotor(dir, pwr,leftMotor);
     setMotor(dir, pwr,rightMotor);
   //store previous error
     eprev=e;

     Serial.print("target: ");
     Serial.print(target);
     Serial.print(" pos1: ");
     Serial.print(pos1);
     Serial.print(" pos2: ");
     Serial.print(pos2);
     Serial.println();
  //}
}



void setMotor(int dir, int pwmVal, Motor m)
{
  //analogWrite(pwm,pwmVal);
  if(dir==-1)
  {
     m.driveForward((uint8_t) pwmVal);
  }
  else if(dir==1)
  {
     m.driveBackward((uint8_t) pwmVal);
  }
  else
  {
     m.stop(); 
  }
}//end of function void setMotor(int dir, int pwmVal, Motor m)


void readEncoder1()
{
   int b=digitalRead(M1_ENC_B);
   if(b>0)
   {
    pos1++;
   }
   else
   {
    pos1--;
   }
}//end of function void readEncoder1()

void readEncoder2()
{
   int b=digitalRead(M2_ENC_B);
   if(b>0)
   {
    pos2++;
   }
   else
   {
    pos2--;
   }
}//end of function void readEncoder2()

/**
 * Return angular speed in degrees / sec
 */
float getAngularSpeed() 
{
  if (!IMU.gyroscopeAvailable()) 
  {
  
    return GYRO_BIAS;
  }
  
  float rotationX, rotationY, rotationZ;
  IMU.readGyroscope(rotationX, rotationY, rotationZ);

  return rotationZ;
}////end of function float getAngularSpeed()
