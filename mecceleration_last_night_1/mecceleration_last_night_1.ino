#include <QTRSensors.h>

#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 38

QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int min_c[8] = {190, 680, 460, 170, 200, 400, 841, 800};
int max_c[8] = {700, 1300, 1000, 550, 600, 1100, 2000, 2000};
#define high 1500
#define turn_high 2000
#define black_line_low 1000

int pre_value;

//PD control
#define kp .3
#define kd 22
#define max_speed 200
#define turn_speed 100
#define base_position 3500
int last_error = 0;
int var_speed;
#define a_speed 100
#define d 300

unsigned int position;

/*Motor*/
#define l_motor_f 2
#define l_motor_b 3
#define r_motor_f 5
#define r_motor_b 4
#define l_pwm 7
#define r_pwm 12

void setup()
{
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop()
{
  digitalWrite(13, LOW);
  position = readline();
  boolean frwrd = false;

  if (sensorValues[0] > turn_high && sensorValues[7] > turn_high && (sensorValues[3] < black_line_low || sensorValues[4] < black_line_low))
  {
    digitalWrite(13, HIGH);
    do
    {
      position = read_black_line();
      follow_line(position);
    } while (sensorValues[7] < black_line_low && sensorValues[0] < black_line_low);
  }
  // left turn
  else if (sensorValues[0] > turn_high && sensorValues[1] > turn_high && sensorValues[2] > turn_high && sensorValues[3] > turn_high && sensorValues[4] > turn_high)
  {
    stop();
    digitalWrite(13, HIGH);
    delay(500);
    go_straight(a_speed);
    do
    {
      position = readline();
      if (sensorValues[7] > turn_high && sensorValues[6] > turn_high)
      {
        frwrd = true;
        break;
      }
    } while (sensorValues[3] > black_line_low || sensorValues[4] > black_line_low);
    if (frwrd)
    {
      go_straight(a_speed);
      do {
        readline();
      } while (sensorValues[0] > black_line_low || sensorValues[7] > black_line_low);
      //      follow_line(position);
    }
    else
      left_turn(d);
  }
  //right turn
  else if (sensorValues[7] > turn_high && sensorValues[6] > turn_high && sensorValues[5] > turn_high && sensorValues[4] > turn_high && sensorValues[3] > turn_high)
  {
    stop();
    digitalWrite(13, HIGH);
    delay(500);
    go_straight(a_speed);
    do
    {
      position = readline();
      if (sensorValues[0] > turn_high && sensorValues[1] > turn_high)
      {
        frwrd = true;
        break;
      }
    } while (sensorValues[3] > black_line_low || sensorValues[4] > black_line_low);
    if (frwrd)
    {
      go_straight(a_speed);
      do {
        readline();
      } while (sensorValues[0] > black_line_low || sensorValues[7] > black_line_low);
      //      follow_line(position);
    }
    else
      right_turn(d);
  }
  else
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
    sensorValues[i] = map(sensorValues[i], min_c[i], max_c[i], 2500, 0);
    //    sensorValues[i] = 2500 - sensorValues[i];
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

void follow_line(unsigned int pos)
{
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, HIGH);
  int error = pos - base_position;
  
  int speed_var = kp * error + kd * (error - last_error);
  /*  Serial1.print(error);
    Serial1.print("   ");
    Serial1.println();
  */
  last_error = error;
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

void left_turn(int del)
{
  digitalWrite(l_motor_b, HIGH);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, LOW);
  digitalWrite(r_motor_f, HIGH);
  analogWrite(l_pwm, turn_speed);
  analogWrite(r_pwm, turn_speed);
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
  analogWrite(l_pwm, turn_speed);
  analogWrite(r_pwm, turn_speed);
  delay(del);
  do {
    readline();
  } while (sensorValues[3] < high && sensorValues[4] < high);
}

void stop()
{
  analogWrite(l_pwm, 0);
  analogWrite(r_pwm, 0);
}

void go_straight( int speed)
{
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, HIGH);
  analogWrite(l_pwm, speed);
  analogWrite(r_pwm, speed);
}

