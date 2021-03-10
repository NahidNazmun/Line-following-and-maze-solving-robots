#include <QTRSensors.h>

//for readline
/*qtrc*/
#define num_sensors 8
#define TIMEOUT 2500
#define EMITTER_PIN 38
//for black line 
int min_black_rc[8] = {570, 600, 430, 230, 240, 410, 560, 300};
int max_black_rc[8] = {2200, 1800, 1200, 690, 740, 1370, 1850, 1100};
//for white line
int min_white_rc[8] = {700, 630, 450, 300, 280, 500, 710, 350};
int max_white_rc[8] = {2300, 2300, 2300, 2150, 2200, 2300, 2300, 2300};
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
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
float kp = .3 ;
float kd = 4 ;
#define max_speed 255
#define turn_speed 200
#define base_position 3500
int last_error = 0;


//for turn
#define high 2000
#define low 500
#define del 300 //turn delay
#define slow_speed 200 // after finding turn it will go ahead with slow speed


//for analog read
int right_sensor;
int left_sensor;
#define right_sensor_low 150
#define right_sensor_high 1000
#define left_sensor_high 750
#define left_sensor_low 100

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

  while (!Serial1.available());

  // for initializing blackline
  if (analogRead(A0) > 300 && analogRead(A1) > 300)
    black_line = false;
  else
    black_line = true;

}

void loop()
{
  bluetooth();
  position = readline();
  analogread();
//  if (left_sensor > high)
//  {
//    if (sensor_values[3] > high || sensor_values[4] > high)
//    {
//      digitalWrite(40, HIGH);
//      digitalWrite(l_motor_f, HIGH);
//      digitalWrite(l_motor_b, LOW);
//      digitalWrite(r_motor_f, HIGH);
//      digitalWrite(r_motor_b, LOW);
//      analogWrite(l_pwm, turn_speed);
//      analogWrite(r_pwm, turn_speed);
//      do {
//        bluetooth();
//        analogread();
//      }while(left_sensor > low);
//      Serial1.println(" left sensor is okay ");
//      do{
//        bluetooth();
//        position =readline();
//      }while(sensor_values[3] > low || sensor_values[4] > low); 
//      Serial1.println("qtrc is okay");
//      left_turn(300);
//    }
//  }
//
//  else if (right_sensor > high)
//  {
//    if (sensor_values[3] > high || sensor_values[4] > high)
//    {
//      digitalWrite(13, HIGH);
//      digitalWrite(l_motor_f, HIGH);
//      digitalWrite(l_motor_b, LOW);
//      digitalWrite(r_motor_f, HIGH);
//      digitalWrite(r_motor_b, LOW);
//      analogWrite(l_pwm, turn_speed);
//      analogWrite(r_pwm, turn_speed);
//      do {
//        bluetooth();
//        analogread();
//      }while(right_sensor>low);
//      Serial1.println(" right sensor is okay ");
//      do{
//        bluetooth();
//        position =readline();
//      }while(sensor_values[3] > low || sensor_values[4] > low); 
//      Serial1.println("qtrc is okay");
//      right_turn(300);
//    }
//  }
//
//  else
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
  } while (sensor_values[5] < high && sensor_values[4] < high);
  digitalWrite(40, LOW);
}

unsigned int readline()
{
  unsigned int pos = 0;
  int sum = 0;
  boolean a = false; //for sensing line
  qtrrc.read(sensor_values);
  for ( int i = 0; i < num_sensors; i++)
  {
    
    if (black_line)
    {
      sensor_values[i] = constrain(sensor_values[i], min_black_rc[i], max_black_rc[i]);
      sensor_values[i] = map(sensor_values[i], min_black_rc[i], max_black_rc[i], 0, 2500);
    }
    else
    {
      sensor_values[i] = constrain(sensor_values[i], min_white_rc[i], max_white_rc[i]);
      sensor_values[i] = map(sensor_values[i], min_white_rc[i], max_white_rc[i], 2500, 0);
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
  right_sensor = constrain(right_sensor, right_sensor_low, right_sensor_high);
  left_sensor = constrain(left_sensor, left_sensor_low, left_sensor_high);
  if (black_line)
  {
    right_sensor = map(right_sensor, right_sensor_low, right_sensor_high, 0, 2500);
    left_sensor = map(left_sensor, left_sensor_low, left_sensor_high, 0, 2500);
  }
  else
  {
    right_sensor = map(right_sensor, right_sensor_low, right_sensor_high, 2500, 0);
    left_sensor = map(left_sensor, left_sensor_low, left_sensor_high, 2500, 0);
  }
}


void bluetooth()
{
  if (Serial1.available() > 0)
  {
    analogWrite(l_pwm, 0);
    analogWrite(r_pwm, 0);
    char a = Serial1.read();
    Serial1.println("stop");
    while (!Serial1.available());
    Serial1.read();
    delay(500);
  }
}
