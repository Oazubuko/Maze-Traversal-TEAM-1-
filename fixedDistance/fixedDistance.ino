#include <Encoder.h>

const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int PWM_VALUE = 255;

const unsigned int ticksPerRotation = 2;

void M1_backward() {
  analogWrite(M1_IN_1, PWM_VALUE);
  analogWrite(M1_IN_2, 0);
}

void M1_forward() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM_VALUE);
}

void M1_stop() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}

void M2_backward() {
  analogWrite(M2_IN_1, PWM_VALUE);
  analogWrite(M2_IN_2, 0);
}

void M2_forward() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM_VALUE);
}

void M2_stop() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}


void setup() 
{
  Serial.begin(115200);

  //analogWrite(2, 255);
  //analogWrite(3, 0);
  //analogWrite(4, 255);
  //analogWrite(5, 0);

  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);
}

void loop() 
{
  M1_stop();
  M2_stop();

  int distanceToTravel=10;//in cm
  float wheelCircumfrence=3.14159265358*2*1.5;//based on wheel radius of 1.5cm
  float numRevolutions=distanceToTravel/wheelCircumfrence;
  Serial.print("numRevolutions=");
  Serial.println(numRevolutions);

  M1_forward();
  M2_forward();
  
  float dist1=0;
  float dist2=0;
  int startDist1=enc1.read();
  int startDist2=enc2.read();
  while((dist1<numRevolutions)&&(dist2<numRevolutions))
  {
  	dist1=(enc1.read()-startDist1)/ticksPerRotation;
    dist2=(enc2.read()-startDist2)/ticksPerRotation;
    Serial.print(dist1);
  	Serial.print("\t");
  	Serial.print(dist2);
  	Serial.println();
  	//delay(10);
  }
  M1_stop();
  M2_stop();
  delay(5000);
}
