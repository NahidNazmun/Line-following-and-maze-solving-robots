#include<digitalWriteFast.h>
unsigned long t = 0;
unsigned long t1 =0;
unsigned long t2 =0;
unsigned long t3 =0;
unsigned long s1 =0;
unsigned long s2 =0;
unsigned long s3 =0;

int led1= 11;
int led2= 10;
int led3=  9;

void setup(void);
void loop(void);

void setup() 
{
  pinModeFast2(led1,OUTPUT);
  pinModeFast(led2,OUTPUT);
  pinMode(led3,OUTPUT);
  Serial.begin(115220);
}

void loop() 
{
  t=micros();
  for(int i=0;i<1000;i++)
  {
    digitalWriteFast2(led1,HIGH);
    digitalWriteFast2(led1,LOW);
  }
  t1=micros();
  s1=t1-t;

  t=micros();
  for(int i=0;i<1000;i++)
  {
    digitalWriteFast(led2,HIGH);
    digitalWriteFast(led2,LOW);
  }
  t2=micros();
  s2=t2-t;

  t=micros();
  for(int i=0;i<1000;i++)
  {
    digitalWrite(led3,HIGH);
    digitalWrite(led3,LOW);
  }
  t3=micros();
  s3=t3-t;
  Serial.print("digitalWriteFast2 = ");
  Serial.println(s1);
  Serial.print("digitalWriteFast = ");
  Serial.println(s2);
  Serial.print("digitalWrite = ");
  Serial.println(s3);
  while(1);
}
