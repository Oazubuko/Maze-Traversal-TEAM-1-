#include <Arduino_LSM9DS1.h>

const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int PWM_VALUE = 75;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float gxsum = 0, prev=micros(), now=0, dt;


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

void setup() {
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
    Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
}

void loop() {
 

  M1_stop();
  M2_stop();

  delay(5000);
  
  M1_forward();
  M2_forward();
  


  for(int i = 0; i < 500; i++) { 
       if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    now = micros();
    dt = (now-prev)/(1e6);
    prev = now;
    if(abs(gz) > 1){
      gxsum = (gz+.5)*dt + gxsum;
    }

  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }

    int M1_I_counts = analogRead(M1_I_SENSE);
    int M2_I_counts = analogRead(M2_I_SENSE);

    Serial.print(M1_I_counts);
    Serial.print("\t");
    Serial.print(M1_I_counts * M_I_COUNTS_TO_A);
    Serial.print("\t");
    Serial.print(M2_I_counts);
    Serial.print("\t");
    Serial.print(M2_I_counts * M_I_COUNTS_TO_A);
    Serial.println();
    delay(1);
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

  Serial.print("Gyro_sum: ");
  Serial.print(gxsum);
  Serial.print('\t');
 M1_forward();
 M2_backward();
  delay(500);
}
