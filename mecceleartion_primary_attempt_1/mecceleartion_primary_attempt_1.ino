#include <QTRSensors.h>

/*qtrc*/
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 38


int min_c[8] = {91, 359, 209, 61, 61, 120, 254, 343};
int max_c[8] = {535, 1508, 860, 296, 296, 565, 1044, 1435};
#define high  1500
#define turn_high 2000
#define black_line_low 1000
int pre_value;

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

//PD control
#define kp .4
#define kd 16
#define max_speed 200
#define base_position 3500
int last_error = 0;
int var_speed;
#define a_speed 200

unsigned int position;

/*Motor*/
#define l_motor_f 9
#define l_motor_b 8
#define r_motor_f 10
#define r_motor_b 11
#define l_pwm 7
#define r_pwm 12

boolean j = false;

//int d = 300;

void setup()
{
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop()
{
  position = read_black_line();

  // black line read
//  if ( sensorValues[0] > turn_high && sensorValues[1] > turn_high && sensorValues[7] > turn_high && sensorValues[6] > turn_high && sensorValues[3] < black_line_low && sensorValues[4] < black_line_low)
//  {
//    do
//    {
//      position = read_black_line();
//      follow_line(position);
//    } while (sensorValues[7] < black_line_low && sensorValues[0] < black_line_low);
//  }
//
//
//  //left turn & loop
//  else if (sensorValues[0] > turn_high && sensorValues[1] > turn_high && sensorValues[2] > turn_high && sensorValues[3] > turn_high && sensorValues[4] > turn_high)
//  {
//    analogWrite(l_pwm, 0);
//    analogWrite(r_pwm, 0);
//    boolean stright = false; // for loop
//    digitalWrite(l_motor_b, LOW);
//    digitalWrite(r_motor_b, LOW);
//    digitalWrite(l_motor_f, HIGH);
//    digitalWrite(r_motor_f, HIGH);
//    analogWrite(l_pwm, a_speed);
//    analogWrite(r_pwm, a_speed);
//    do
//    {
//      if (sensorValues[7] > turn_high && sensorValues[6] > turn_high)
//      {
//        stright = true;
//      }
//      position = readline();
//    } while (sensorValues[0] > turn_high && sensorValues[1] > turn_high);
//    if (stright)
//    {
//      follow_line(position);
//    }
//    else
//      left_turn(300);
//  }
//
//  //right turn & loop
//  else if (sensorValues[7] > turn_high && sensorValues[6] > turn_high && sensorValues[5] > turn_high && sensorValues[4] > turn_high && sensorValues[3] > turn_high)
//  {
//    analogWrite(l_pwm, 0);
//    analogWrite(r_pwm, 0);
//    boolean stright = false; // for loop
//    digitalWrite(l_motor_b, LOW);
//    digitalWrite(r_motor_b, LOW);
//    digitalWrite(l_motor_f, HIGH);
//    digitalWrite(r_motor_f, HIGH);
//    analogWrite(l_pwm, a_speed);
//    analogWrite(r_pwm, a_speed);
//    do
//    {
//      if (sensorValues[0] > turn_high && sensorValues[1] > turn_high)
//      {
//        stright = true;
//      }
//      position = readline();
//    } while (sensorValues[7] > turn_high && sensorValues[6] > turn_high);
//    if (stright)
//    {
//      follow_line(position);
//    }
//    else
//      right_turn(300);
//  }
//
//  //broken part
//  else if (sensorValues[0] < black_line_low && sensorValues[1] < black_line_low  && sensorValues[3] < black_line_low && sensorValues[4] < black_line_low && sensorValues[6] < black_line_low && sensorValues[7] < black_line_low && position > 2500 && position <4500)
//  {
//    digitalWrite(l_motor_b, LOW);
//    digitalWrite(r_motor_b, LOW);
//    digitalWrite(l_motor_f, HIGH);
//    digitalWrite(r_motor_f, HIGH);
//    analogWrite(l_pwm, max_speed);
//    analogWrite(r_pwm, max_speed);
//  }
//  
//  else
    follow_line(position);
}




unsigned int readline()
{
  unsigned int pos = 0;
  int sum = 0;
  boolean a = false;
  qtrrc.read(sensorValues);
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    sensorValues[i] = constrain(sensorValues[i], min_c[i], max_c[i]);
    sensorValues[i] = map(sensorValues[i], min_c[i], max_c[i], 0, 2500);
    sensorValues[i] = 2500 - sensorValues[i];
    if (sensorValues[i] > high)
      a = true;
    pos += (i * 1000 * sensorValues[i]);
    sum += sensorValues[i];
  }
  if (a)
  {
    pos = pos / sum;
    pre_value = pos;
  }
  else
  {
    if (pre_value > 6000)
      pos = (NUM_SENSORS - 1) * 1000;
    else if (pre_value < 1000)
      pos = 0;
    else
      pos = pre_value;
  }
  return pos;
}

unsigned int read_black_line()
{
  unsigned int pos = 0;
  int sum = 0;
  boolean a = false;
  qtrrc.read(sensorValues);
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    sensorValues[i] = constrain(sensorValues[i], min_c[i], max_c[i]);
    sensorValues[i] = map(sensorValues[i], min_c[i], max_c[i], 0, 2500);
    if (sensorValues[i] > high)
      a = true;
    pos += (i * 1000 * sensorValues[i]);
    sum += sensorValues[i];
  }
  if (a)
  {
    pos = pos / sum;
    pre_value = pos;
  }
  else
  {
    if (pre_value > 6000)
      pos = (NUM_SENSORS - 1) * 1000;
    else if (pre_value < 1000)
      pos = 0;
    else
      pos = pre_value;
  }
  return pos;
}

void bluetooth()
{
  if (Serial1.available() > 0)
  {
    analogWrite(l_pwm, 0);
    analogWrite(r_pwm, 0);
    char a = Serial1.read();
    Serial1.println("delays");
    while (!Serial1.available());
    Serial1.read();
    delay(500);
  }
}

void left_turn(int del)
{
  digitalWrite(l_motor_b, HIGH);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, LOW);
  digitalWrite(r_motor_f, HIGH);
  analogWrite(l_pwm, max_speed);
  analogWrite(r_pwm, max_speed);
  delay(del);
  do {
    readline();
  } while (sensorValues[3] < high && sensorValues[4] < high);
}

void right_turn(int del)
{
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, HIGH);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, LOW);
  analogWrite(l_pwm, max_speed);
  analogWrite(r_pwm, max_speed);
  delay(del);
  do {
    readline();
  } while (sensorValues[3] < high && sensorValues[4] < high);
}

void follow_line(unsigned int pos)
{
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, HIGH);
  int error = pos - base_position;
  last_error = error;
  int speed_var = kp * error + kd * (error - last_error);
  /*  Serial1.print(error);
    Serial1.print("   ");
    Serial1.println();
  */
  if (speed_var < 0) //left turn
  {
    analogWrite(l_pwm, max(0, max_speed + speed_var));
    analogWrite(r_pwm, max_speed);
  }
  else
  {
    analogWrite(r_pwm, max(0, max_speed - speed_var));
    analogWrite(l_pwm, max_speed);
  }
}
