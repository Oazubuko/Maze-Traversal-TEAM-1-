/*********************************** Test 5******************************/

#define M1_ENC_A 6 //
#define M1_ENC_B 7 //
#define M2_ENC_A 8 //
#define M2_ENC_B 9 //

#define MAX_PWM  50
#define M1_IN_1  2
#define M1_IN_2  3
#define M2_IN_1  5
#define M2_IN_2  4

int   pos1      = 0;
int   pos2      = 0;
long  prevT     = 0;
float eprev     = 0;
float eintegral = 0;

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder1();
void readEncoder2();

int flag=0;

void setup()
{
 Serial.begin(9600);
 pinMode(M1_ENC_A,INPUT);
 pinMode(M1_ENC_B,INPUT);
 pinMode(M2_ENC_A,INPUT);
 pinMode(M2_ENC_B,INPUT);
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
     setMotor(dir, pwr,M1_IN_1,M1_IN_2);
     setMotor(dir, pwr,M2_IN_1,M2_IN_2);
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



void setMotor(int dir, int pwmVal, int in1, int in2)
{
  //analogWrite(pwm,pwmVal);
  if(dir==1)
  {
  //digitalWrite(in1,HIGH);
  //digitalWrite(in2,LOW);
     analogWrite(in1,pwmVal);
     analogWrite(in2,0); 

  }
  else if(dir==-1)
  {
  //digitalWrite(in1,LOW);
  //digitalWrite(in2,HIGH);
     analogWrite(in1,0);
     analogWrite(in2,pwmVal); 

  }
  else
  {
  //digitalWrite(in1,LOW);
  //digitalWrite(in2,LOW);
     analogWrite(in1,0);
     analogWrite(in2,0); 
  }
}


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
}

void readEncoder2()
{
   int b=digitalRead(M2_ENC_A);
   if(b>0)
   {
    pos2++;
   }
   else
   {
    pos2--;
   }
}
