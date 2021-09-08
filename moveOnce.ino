#include <Arduino_LSM9DS1.h>

const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;
int flag=0;

const unsigned int PWM_VALUE = 127;

void M1_backward() 
{
  analogWrite(M1_IN_1, PWM_VALUE);
  analogWrite(M1_IN_2, 0);
}

void M1_forward() 
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM_VALUE);
}

void M1_stop() 
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}

void M2_backward() 
{
  analogWrite(M2_IN_1, PWM_VALUE);
  analogWrite(M2_IN_2, 0);
}

void M2_forward() 
{
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM_VALUE);
}

void M2_stop() 
{
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}

void setup() 
{
  // put your setup code here, to run once:
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if(!flag)
  {
    float delta=0;
    float velocity=0;
    int t1=0,t2=0;
    float ax, ay, az;

    Serial.print("delta: ");
    Serial.print(delta);
    Serial.print('\t');
    Serial.print("velocity: ");
    Serial.print(velocity);
    Serial.print('\t');
    Serial.print("t1: ");
    Serial.print(t1);
    Serial.print('\t');
    Serial.print("t2: ");
    Serial.println(t2);

    M1_stop();
    M2_stop();


    if (IMU.accelerationAvailable()) 
    {
      IMU.readAcceleration(ax, ay, az);
    }

    Serial.print("Acc: ");
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.println(az);

    t1=millis();
    Serial.print("After millis() t1: ");
    Serial.println(t1);
    
    M1_forward();
    M2_forward();

    delay(501);
    
    if (IMU.accelerationAvailable()) 
    {
      IMU.readAcceleration(ax, ay, az);
    }

    Serial.print("Acc: ");
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.println(az);

    delay(500);

    t2=millis();
    Serial.println("After millis() t2: ");
    Serial.println(t2);

    M1_stop();
    M2_stop();

    flag=1;
  }
}
