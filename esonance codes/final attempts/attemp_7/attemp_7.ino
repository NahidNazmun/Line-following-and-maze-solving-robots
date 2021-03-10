#include <QTRSensors.h>

//for readline
/*qtrc*/
#define num_sensors 8
#define TIMEOUT 1500
#define EMITTER_PIN 38
int min_rc[8] = {158, 101, 115, 173, 116, 87, 101, 172};
int max_rc[8] = {1332, 782, 877, 1167, 783, 590, 727, 1264};
QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
}, // 0 no sensor= right
num_sensors, TIMEOUT, EMITTER_PIN);
unsigned int sensor_values[num_sensors];

int pre_value = 3500;
#define readline_high  1300
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
#define turn_speed 120
#define base_position 3500
int last_error = 0;


//for turn
int a_right_high;
int a_right_low;
int a_left_high;
int a_left_low;
int high;
#define low 500
#define del 300 //turn delay
#define slow_speed 200 // after finding turn it will go ahead with slow speed


//for analog read
int right_sensor;
int left_sensor;


//for white line analog sensor calibaration
#define white_left_high 800
#define white_left_low 200
#define white_right_high 800
#define white_right_low 100
#define white_qtr_high 1800

//for black line analog sensor calibration
#define black_left_high 600
#define black_left_low 100
#define black_right_high 800
#define black_right_low 100
#define black_qtr_high 1500

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

  while (!Serial1.available());

  // for initializing blackline
  if (analogRead(A0) > 500 && analogRead(A1) > 500)
  {
    black_line = false;
    a_left_high = white_left_high;
    a_left_low = white_left_low;
    a_right_high = white_right_high;
    a_right_low = white_right_low;
    high = white_qtr_high;
  }
  else
  {
    black_line = true;
    a_left_high = black_left_high;
    a_left_low = black_left_low;
    a_right_high = black_right_high;
    a_right_low = black_right_low;
    high = black_qtr_high;
  }
}

void loop()
{
start:
  bluetooth();
  analogread();
  position = readline();

  if (left_sensor > a_left_high && (sensor_values[3] > high || sensor_values[4] > high))
  {
    digitalWrite(l_motor_f, HIGH);
    digitalWrite(l_motor_b, LOW);
    digitalWrite(r_motor_f, HIGH);
    digitalWrite(r_motor_b, LOW);
    analogWrite(l_pwm, turn_speed);
    analogWrite(r_pwm, turn_speed);
    do {
      bluetooth();
      analogread();
      if (right_sensor > a_right_high && position > 2000 && position < 5000)
      {
tsection:
        unsigned long tm = millis();
        unsigned long tm1;
        do {
          bluetooth();
          analogread();
          tm1 = millis();
          if ((tm1 - tm) > 300)
          {
            black_line = !(black_line);
            if (black_line)
            {
              Serial1.println("Switched to black");
              a_left_high = black_left_high;
              a_left_low = black_left_low;
              a_right_high = black_right_high;
              a_right_low = black_right_low;
              high = black_qtr_high;
            }
            else
            {
              Serial1.println("Switched to white");
              a_left_high = white_left_high;
              a_left_low = white_left_low;
              a_right_high = white_right_high;
              a_right_low = white_right_low;
              high = white_qtr_high;
            }
            analogWrite(l_pwm, max_speed);
            analogWrite(r_pwm, max_speed);
            delay(100);
            goto start;
          }
        } while (left_sensor > a_left_low && right_sensor > a_right_low);
      }
    } while (left_sensor > a_left_low);
    do {
      bluetooth();
      position = readline();
      if (sensor_values[7] > high && sensor_values[6] > high && sensor_values[5] > high)
      {
        left_turn(300);
        break;
      }
    } while (sensor_values[3] > low || sensor_values[4] > low);
    left_turn(0);
  }

  else if (right_sensor > a_right_high && (sensor_values[3] > high || sensor_values[4] > high))
  {
    digitalWrite(l_motor_f, HIGH);
    digitalWrite(l_motor_b, LOW);
    digitalWrite(r_motor_f, HIGH);
    digitalWrite(r_motor_b, LOW);
    analogWrite(l_pwm, turn_speed);
    analogWrite(r_pwm, turn_speed);
    do {
      bluetooth();
      analogread();
      if (left_sensor > a_left_high && position > 2000 && position < 5000)
        goto tsection;
    } while (right_sensor > a_right_low);
    do {
      bluetooth();
      position = readline();
      if (sensor_values[0] > high && sensor_values[1] > high && sensor_values[2] > high)
      {
        right_turn(300);
        break;
      }
    } while (sensor_values[3] > low || sensor_values[4] > low);
    right_turn(0);
  }

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
  } while (sensor_values[5] < high && sensor_values[4] < high);
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
  } while (sensor_values[3] < high && sensor_values[2] < high);
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
      pos = pre_value;
  }
  return pos;
}

void analogread()
{
  right_sensor = analogRead(A0); //right
  left_sensor = analogRead(A1);
  if (!black_line)
  {
    right_sensor = map(right_sensor, 0, 1023, 1023, 0);
    left_sensor = map(left_sensor, 0, 1023, 1023, 0);
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
