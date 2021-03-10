#include <QTRSensors.h>

//for readline
/*qtrc*/
#define num_sensors 8
#define TIMEOUT 1500
#define EMITTER_PIN 38
int min_rc[8] = {289, 190, 189, 371, 165, 147, 146, 280};
int max_rc[8] = {1864, 1277, 1251, 2124, 1058, 981, 956, 1696};
QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
}, // 0 no sensor= right
num_sensors, TIMEOUT, EMITTER_PIN);
unsigned int sensor_values[num_sensors];

int pre_value = 3500;
#define readline_high  1500
boolean black_line;
unsigned int position;


//follow line
/*Motor*/
#define l_motor_f 4
#define l_motor_b 3
#define r_motor_f 5
#define r_motor_b 6
#define l_pwm 2
#define r_pwm 7

/* pd control*/
#define kp  .3
#define kd  4
#define max_speed 255
#define turn_speed 150
#define base_position 3500
int last_error = 0;


//for turn
#define high 1500
#define low 500
#define del 300 //turn delay
#define slow_speed 200 // after finding turn it will go ahead with slow speed


//for analog read
int right_sensor;
int left_sensor;
#define right_sensor_low 150
#define right_sensor_high 960
#define left_sensor_high 1023
#define left_sensor_low 0

void setup()
{
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(44, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  digitalWrite(44, LOW);

//  while (!Serial1.available());

  // for initializing blackline
  if (analogRead(A0) > 500 && analogRead(A1) > 500)
    black_line = false;
  else
    black_line = true;

}

void loop()
{
  bluetooth();
  analogread();
start:
  position = readline();
//  if (left_sensor > 550)
//  {
//    if (sensor_values[3] > high || sensor_values[4] > high)
//    {
//      digitalWrite(l_motor_f, HIGH);
//      digitalWrite(l_motor_b, LOW);
//      digitalWrite(r_motor_f, HIGH);
//      digitalWrite(r_motor_b, LOW);
//      analogWrite(l_pwm, turn_speed);
//      analogWrite(r_pwm, turn_speed);
//      do {
//        bluetooth();
//        analogread();
//      } while (left_sensor > 100);
//      do {
//        bluetooth();
//        position = readline();
//        if (sensor_values[7] > high && sensor_values[6] > high && sensor_values[5] > high)
//        {
//          left_turn(300);
//          break;
//        }
//      } while (sensor_values[3] > low || sensor_values[4] > low);
//      left_turn(0);
//    }
//  }
  follow_line();
}

void follow_line()
{
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_f, HIGH);
  digitalWrite(r_motor_b, LOW);
  int error = position - base_position;
  int speed_var = kp * error + kd * (error - last_error);
  last_error = error;
  if (speed_var > 0) // left turn
  {
    analogWrite(l_pwm, max(0, (max_speed - speed_var))); // for avoiding negative
    analogWrite(r_pwm, max_speed);
  }
  else // right turn
  {
    analogWrite(r_pwm, max(0, (max_speed + speed_var)));
    analogWrite(l_pwm, max_speed);
  }
}

void right_turn(int turn_delay)
{
  digitalWrite(r_motor_b, HIGH);
  digitalWrite(r_motor_f, LOW);
  digitalWrite(l_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  analogWrite(l_pwm, turn_speed);
  analogWrite(r_pwm, turn_speed);
  delay(turn_delay);
  do {
    bluetooth();
    position = readline();
  } while (sensor_values[3] < high && sensor_values[2] < high);
  digitalWrite(13, LOW);
}

void left_turn(int turn_delay)
{
  digitalWrite(l_motor_b, HIGH);
  digitalWrite(l_motor_f, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(r_motor_f, HIGH);
  analogWrite(l_pwm, turn_speed);
  analogWrite(r_pwm, turn_speed);
  delay(turn_delay);
  do {
    bluetooth();
    position = readline();
  } while (sensor_values[3] < high && sensor_values[4] < high);
}

unsigned int readline()
{
  unsigned int pos = 0;
  int sum = 0;
  boolean a = false; //for sensing line
  qtrrc.read(sensor_values);
  for ( int i = 0; i < num_sensors; i++)
  {
    sensor_values[i] = constrain(sensor_values[i], min_rc[i], max_rc[i]);
    if (black_line)
    {
      sensor_values[i] = map(sensor_values[i], min_rc[i], max_rc[i], 0, 2500);
    }
    else
    {
      sensor_values[i] = map(sensor_values[i], min_rc[i], max_rc[i], 2500, 0);
    }
    if (sensor_values[i] > readline_high)
      a = true; //found line
    pos += (i * 1000 * sensor_values[i]);
    sum += sensor_values[i];
  }
  if (a)
  {
    pos = pos / sum;
    pre_value = pos;
  }

  else
  {
    if (pre_value > 6000)
      pos = (num_sensors - 1) * 1000;
    else if (pre_value < 1000)
      pos = 0;
    else
      pos = 3500;
  }
  return pos;
}

void analogread()
{
  right_sensor = analogRead(A0); //right
  left_sensor = analogRead(A1);
  if (!black_line)
  {
    right_sensor = map(right_sensor, 0,1023,1023,0);
    left_sensor = map(left_sensor,0,1023,1023,0);
  }
}


void bluetooth()
{
  if (Serial1.available() > 0)
  {
    analogWrite(l_pwm, 0);
    analogWrite(r_pwm, 0);
    Serial1.read();
    Serial1.println("stop");
    while (!Serial1.available());
    Serial1.read();
    delay(500);
  }
}
