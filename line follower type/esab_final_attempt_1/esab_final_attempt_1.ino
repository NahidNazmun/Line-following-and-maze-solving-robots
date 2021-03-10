#include<QTRSensors.h>
#include<EEPROM.h>

#define num_sensors 6
#define sample_rate 4
#define emitter_pin 255

QTRSensorsAnalog qtra((unsigned char[]) {  0, 1, 2, 3, 4, 5}, num_sensors, sample_rate, emitter_pin);
unsigned int sensor_values[num_sensors];
unsigned int position = 2500;

#define high 750
#define low 100

//motor
#define left_motor_f 6
#define left_motor_b 10
#define right_motor_f 5
#define right_motor_b 9

#define led_pin 13
// pd control
#define kp 1.3//.877
#define kd 13 //7
#define turn_speed 60 //75
#define back_turn_speed 80 //100*
#define max_speed 100
#define move_ahead_speed 120
#define base_position 2500
int last_error = 0;

//check_node
boolean checkpoint = false;
boolean node = false;
boolean uturn = false;
boolean left = false;
#define run_delay 70 //120*
#define turn_delay 400


// read_maze
int path[100];
int node_number = 0;
int p = 0; // path length
boolean state = false;

void setup()
{
  pinMode(left_motor_f, OUTPUT);
  pinMode(left_motor_b, OUTPUT);
  pinMode(right_motor_f, OUTPUT);
  pinMode(right_motor_b, OUTPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  Serial.begin(9600);
  digitalWrite(left_motor_f, 1);
  digitalWrite(right_motor_f, 0);
  digitalWrite(left_motor_b, 0);
  digitalWrite(right_motor_b, 1);
  for(int i = 0; i < 400; i++)
  {
    qtra.calibrate();
  }
  digitalWrite(left_motor_b, 0);
  digitalWrite(right_motor_b, 0);
  digitalWrite(left_motor_b, 0);
  digitalWrite(right_motor_b, 0);
  delay(3000);// this needs to be opted out in the main competition
  digitalWrite(led_pin, LOW);
  read_maze();
  turn_left(600);
  read_maze();
  digitalWrite(left_motor_b, 0);
  digitalWrite(right_motor_b, 0);
  digitalWrite(left_motor_b, 0);
  digitalWrite(right_motor_b, 0);
  
}
void loop()
{
  digitalWrite(13,HIGH);
  delay(250);
  digitalWrite(13,LOW);
  delay(250);  
}

void check_node()
{
  node = false;
  checkpoint = false;
  uturn = false;
  left=false;
  position = qtra.readLine(sensor_values);
  if(sensor_values[0]> high) // left sensor on black line
    left=true;
  //check for uturn
  if(sensor_values[1]<low && sensor_values[2]<low && sensor_values[3]<low && sensor_values[4]<low)
  uturn = true;
  else if(sensor_values[0]>high && sensor_values[1]>high && sensor_values[2]>high && sensor_values[3]>high && sensor_values[4]>high && sensor_values[5]>high)
  {
    delay(run_delay);
    position = qtra.readLine(sensor_values);
    // for checkpoint
    if(sensor_values[0]>high && sensor_values[1]>high && sensor_values[2]>high && sensor_values[3]>high && sensor_values[4]>high && sensor_values[5]>high)
    {
      checkpoint = true;
      return;
    }
    else
    {
      node = true;
      return;
    }
  }
  else if(sensor_values[0]>high || sensor_values[5]>high)
  {
    int temp = position;
    delay(run_delay);
    position = qtra.readLine(sensor_values);
    if(position>2000 && position <3000)
    {
      node = true;
      return;
    }
    else
    {
      position = temp;
      return;       
    }
  }  
}

void turn_right()
{
  analogWrite(right_motor_b,max_speed);
  analogWrite(right_motor_f,0);
  analogWrite(left_motor_f,max_speed);
  analogWrite(left_motor_b,0);

  do
  {
    sensor_values[5] = analogRead(A5);
    sensor_values[4] = analogRead(A4); 
  }while(sensor_values[5]>low && sensor_values[4]>low);
  return;  
}

void turn_left()
{
  analogWrite(right_motor_b,0);
  analogWrite(right_motor_f,max_speed);
  analogWrite(left_motor_f,0);
  analogWrite(left_motor_b,max_speed);
  //run past
  do
  {
    sensor_values[0] = analogRead(A0);
    sensor_values[1] = analogRead(A1); 
  }while(sensor_values[0]>low && sensor_values[1]>low);
  return;  
}

void follow_line()
{
  int error = position - base_position;
  last_error = error;
  int speed_var = kp * error + kd * (error-last_error);

  if(speed_var<0)//left turn
  {
    analogWrite(left_motor_f,(max_speed+speed_var));
    analogWrite(left_motor_b,0);
    analogWrite(right_motor_f,max_speed);
    analogWrite(right_motor_b,0);
  }
  else // right turn
  {
    analogWrite(left_motor_f,max_speed);
    analogWrite(left_motor_b,0);
    analogWrite(right_motor_f,max_speed-speed_var);
    analogWrite(right_motor_b,0);
  }
}

void read_maze()
{
  while(1)
  {
    check_node();
    if(checkpoint)
    {
      analogWrite(right_motor_b,0);
      analogWrite(right_motor_f,150);// to be checked
      analogWrite(left_motor_f,0);
      analogWrite(left_motor_b,150);// to be checked
      p--; // to reduce the extra p we calculated
      find_solution(); // this function is supposed to take some time, otherwise use delay()
      return;    
    }
    else if(uturn)
    {
      turn_left();
      path[p]=180; //180 degree repesents uturn
      p++;
    }
    else if(node)
    {
      if(left)
      {
        turn_left();
        path[p]=90; //90 degree repesents left turn
        p++;
      }
      else
      {
        follow_line();
        path[p]=0;
        p++;        
      }
    }
    else
    follow_line();
  }
}

void find_solution()
{
  boolean t;
  do
  {
    t= false;
    for(int j=1; j<p; j++)
    {
      if(path[j]==180)
      {
        path[j-1]+=(path[j+1]+180);
        if(path[j-1]>360)
          path[j-1]-=360;
        if(path[j-1]==360)
          path[j-1]=0;
        if(path[j-1]==180)
          t=true;
        for(int k=j+2; k<p; k++)
          path[k-2]= path[k];
        p-=2;
        j--;
      }
    }
  }while(t);
}

void flash()
{
  while(1)
  {
    check_node();
    if(checkpoint)
    {
      analogWrite(right_motor_b,0);
      analogWrite(right_motor_f,0);
      analogWrite(left_motor_f,0);
      analogWrite(left_motor_b,0);
      return;
    }
    else if(node)
    {
      switch(path[p])
      {
        case 90:
          turn_right(); // have to turn right because of reverse direction
          break;
        case 270:
          turn_left(); // have to turn left because of reverse direction
          break;
        default:
          follow_line();
          break;
      }
      p--;
    }
    else
    follow_line();
  }
}
