#include<digitalWriteFast.h>

// motor pins
#define left_motor 9
#define right_motor 10

// ultrasonic sensor
#define trig 6
#define echo 5
#define max_dist 2000
unsigned long dist = max_dist;
unsigned long last_dist = max_dist;

//PD control
#define kp .1
#define kd 2
#define max_speed 100
#define base_position 750
int last_error = 0;

void setup() 
{
  pinModeFast2(trig,OUTPUT);
  pinModeFast2(echo,INPUT);
  pinMode(left_motor,OUTPUT);
  pinMode(right_motor,OUTPUT);
  Serial.begin(9600);
}

void loop() 
{ 
  // sensor check
  digitalWriteFast2(trig,HIGH); // these will create the pulse required by the sensor
  delayMicroseconds(12);
  digitalWriteFast2(trig,LOW);
  
  last_dist = dist;
  dist = pulseIn(echo,HIGH,max_dist); // this timeout will limit the max distance read, lower value will decrease dealy between two readings.  
  if (dist == 0)
  {
    if(last_dist > max_dist/3)
    dist = max_dist;
  }
  
  //PD control
  int error = dist-base_position;
  int speed_var = kp*error + kd*(error - last_error);
  last_error = error;

  if(speed_var < 0) // left turn
  {
    analogWrite(left_motor, max(0,(max_speed+speed_var)));
    analogWrite(right_motor, max_speed);
  }
  else // right turn
  {
    analogWrite(left_motor,max_speed);
    analogWrite(right_motor, max(0,(max_speed - speed_var)));
  }
}
