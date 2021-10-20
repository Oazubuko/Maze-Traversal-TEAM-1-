/*
  Arduino LSM9DS1 - Simple Accelerometer
  This example reads the acceleration values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.
  The circuit:
  - Arduino Nano 33 BLE Sense
  created 10 Jul 2019
  by Riccardo Rizzo
  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
BLEService mazeService("0fe79935-cd39-480a-8a44-06b70f36f248");

// create switch characteristic and allow remote device to read and write
BLEUnsignedCharCharacteristic flagCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24a", BLERead | BLEWrite | BLENotify);
BLEStringCharacteristic directionsCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24c", BLERead | BLEWrite,100);
String directions="LBLLLBSBLLBSLL";

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float gzsum = 0, prev=micros(), now=0, dt, tol=0;
float axsum = 0, aysum = 0, axsum1 = 0, aysum1 = 0, dx, dy;

void setup() 
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) 
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }//End of if (!IMU.begin()) 

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
}//End of void Setup




void loop() 
{
  
  if (IMU.accelerationAvailable()) 
  { 
    IMU.readAcceleration(ax, ay, az);
  }//End of if (IMU.accelerationAvailable())

  if (IMU.gyroscopeAvailable()) 
  {
    IMU.readGyroscope(gx, gy, gz);
    now = micros();
    dt = (now-prev)/(1e6);
    prev = now;
    
    axsum1 = (ax+.5)*dt + axsum;
    axsum = (ay+.5)*dt + axsum;
    dx = abs((axsum1-axsum)/dt);
    
    aysum1 = (ay+.5)*dt + aysum;
    aysum = (ay+.5)*dt + aysum;
    dy = abs((aysum1-aysum)/dt);
    
    if(abs(gz) > 1)
    {
      gzsum = (gz+.5)*dt + gzsum;
    }//if(abs(gz) > 1)
    if((axsum > tol & aysum > tol) | gzsum > tol)
    {
       flagCharacteristic.setValue(1);
    }//End of if((mxsum > tol & mysum > tol) | gzsum > tol)

  }//if (IMU.gyroscopeAvailable())

  if (IMU.magneticFieldAvailable()) 
  {
    IMU.readMagneticField(mx, my, mz);
  }

  Serial.print("Acc: ");
  Serial.print(ax);
  Serial.print('\t');
  Serial.print(ay);
  Serial.print('\t');
  Serial.print(az);
  Serial.print('\t');

  Serial.print("Gyr: ");
  Serial.print(gx);
  Serial.print('\t');
  Serial.print(gy);
  Serial.print('\t');
  Serial.print(gz);
  Serial.print('\t');

  Serial.print("Mag: ");
  Serial.print(mx);
  Serial.print('\t');
  Serial.print(my);
  Serial.print('\t');
  Serial.println(mz);

  Serial.print("Acc_sum: ");
  Serial.print(dx);
  Serial.print('\t');
  
}//End of Loop
