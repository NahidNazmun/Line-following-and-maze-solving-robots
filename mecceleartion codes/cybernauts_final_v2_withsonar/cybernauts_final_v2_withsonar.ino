#include <digitalWriteFast.h>
#include <QTRSensors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

#define high 2500
#define low 200
#define b_high 2000
#define b_low 500


// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
unsigned int lastValues[NUM_SENSORS];
unsigned int lastposition;

//PD control
#define kp .1
#define kd 2
#define base_position 3500
int last_error = 0;

// motor pins
#define left_motor_f 9
#define right_motor_f 10
#define left_motor_b 25
#define right_motor_b 26

#define max_speed 100
#define turn_speed 120
#define a_speed 50

unsigned int position;

// ultrasonic sensor
#define trig 6
#define echo 5
#define max_dist 2000

unsigned long dist = max_dist;
unsigned long last_dist = max_dist;

//sonar pid
#define s_kp .1
#define s_kd 2
#define s_max_speed 100
#define s_base_position 750
int s_last_error = 0;

void setup() {
  pinMode(left_motor_f, OUTPUT);
  pinMode(right_motor_f, OUTPUT);
  pinMode(left_motor_b, OUTPUT);
  pinMode(right_motor_b, OUTPUT);
}

void loop() {
begining:
  digitalWrite(left_motor_b, LOW);
  digitalWrite(right_motor_b, LOW);
  position = readline();

  if (sensorValues[1] < b_low && sensorValues[2] < b_low && sensorValues[3] < b_low && sensorValues[4] < b_low && sensorValues[5] < b_low && sensorValues[6] < b_low)
  {
    //sonar sensor part
    dist = sonar_read();
    if (dist != max_dist)
    {
      do
      {
        position = readline();
        dist = sonar_read();
        wall_follow(dist);
      } while (sensorValues[1] < b_high && sensorValues[2] < b_high && sensorValues[3] < b_high && sensorValues[4] < b_high &&  sensorValues[5] < b_high && sensorValues[6] < b_high);
    }
    else
      line_follow(position);
  }

  // checkpoint and t-section
  else if (sensorValues[0] > b_high && sensorValues[1] > b_high && sensorValues[2] > b_high && sensorValues[5] > b_high && sensorValues[6] > b_high && sensorValues[7] > b_high)
  {
    analogWrite(left_motor_f, a_speed);
    analogWrite(right_motor_f, a_speed);
    delay(300); //need to be check
    position = readline();
    if (sensorValues[0] > b_high && sensorValues[1] > b_high && sensorValues[2] > b_high && sensorValues[5] > b_high && sensorValues[6] > b_high && sensorValues[7] > b_high)
    {
      while (1)
      {
        digitalWrite(left_motor_f, LOW);
        digitalWrite(right_motor_f, LOW);
      }
    }
    else //t-section
      goto begining;
  }

  //left turn
  else if (sensorValues[0] > b_high && sensorValues[1] > b_high && sensorValues[2] > b_high && sensorValues[3] > b_high)
  {
    digitalWrite(left_motor_f, LOW);
    digitalWrite(right_motor_f, LOW);
    delay(50);
    do
    {
      analogWrite(right_motor_f, a_speed);
      analogWrite(left_motor_f, a_speed);
      position = readline();
    } while (sensorValues[0] > b_high);
    left_turn();
  }

  //right turn need to check this part
  else if (sensorValues[7] > b_high && sensorValues[6] > b_high && sensorValues[5] > b_high && sensorValues[4] > b_high)
  {
    digitalWrite(left_motor_f, LOW);
    digitalWrite(right_motor_f, LOW);
    delay(50);
    do
    {
      analogWrite(right_motor_f, a_speed);
      analogWrite(left_motor_f, a_speed);
      position = readline();
      if (sensorValues[0] > b_high && sensorValues[1] > b_high)
      {
        left_turn();
        goto begining;
      }
    } while (sensorValues[7] > b_high);
    right_turn();
  }

  //straight line
  else
    line_follow(position);

}


unsigned int readline()
{
  qtrrc.read(sensorValues);
  unsigned int pos = 0;
  int sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] < low)
      sensorValues[i] = 0;
    else if (sensorValues[i] < high)
      sensorValues[i] = sensorValues[i] - low;
    else
      sensorValues[i] = high - low;
    pos = pos + (i * 1000) * sensorValues[i];
    sum = sum + sensorValues[i];
  }

  for (int j = 0; j < NUM_SENSORS; j++)
  {
    if (sensorValues[j] > b_high)  // critical point
      goto label;
  }
  pos = lastposition;
  return pos;
label:
  for (int j = 0; j < NUM_SENSORS; j++)
  {
    lastValues[NUM_SENSORS] = sensorValues[NUM_SENSORS];
  }
  pos = pos / sum;
  lastposition = pos;
  return pos;
}

void line_follow(unsigned int pos)
{
  digitalWrite(left_motor_b, LOW);
  digitalWrite(right_motor_b, LOW);

  int error = pos - base_position;
  int speed_var = kp * error + kd * (error - last_error);
  last_error = error;
  if (speed_var > 0) //left turn
  {
    analogWrite(left_motor_f, (max(0, max_speed - speed_var)));
    analogWrite(right_motor_f, max_speed);
  }
  else // right turn
  {
    analogWrite(left_motor_f, max_speed);
    analogWrite(right_motor_f, (max(0, max_speed + speed_var)));
  }
}

void left_turn()
{
  digitalWrite(left_motor_f, LOW);
  digitalWrite(right_motor_b, LOW);
  analogWrite(left_motor_b, turn_speed);
  analogWrite(right_motor_f, turn_speed);
  delay(300);
  do
  {
    analogWrite(left_motor_b, turn_speed);
    analogWrite(right_motor_f, turn_speed);
    position = readline();
  } while (sensorValues[3] < b_high && sensorValues[4] < b_high);
}

void right_turn()
{
  digitalWrite(left_motor_b, LOW);
  digitalWrite(right_motor_f, LOW);
  analogWrite(left_motor_f, turn_speed);
  analogWrite(right_motor_b, turn_speed);
  delay(300);
  do
  {
    analogWrite(left_motor_f, turn_speed);
    analogWrite(right_motor_b, turn_speed);
    position = readline();
  } while (sensorValues[3] < b_high && sensorValues[4] < b_high);
}

unsigned long sonar_read()
{
  // sensor check
  digitalWriteFast2(trig, HIGH); // these will create the pulse required by the sensor
  delayMicroseconds(10);
  digitalWriteFast2(trig, LOW);

  unsigned long distance;
  dist = pulseIn(echo, HIGH, max_dist); // this timeout will limit the max distance read, lower value will decrease dealy between two readings.
  if (dist == 0)
  {
    //    if(last_dist > 500)
    distance = max_dist;
  }
  return distance;
}

void wall_follow(unsigned long distance) //need to upgrade this section
{
  digitalWrite(left_motor_b, LOW);
  digitalWrite(right_motor_b, LOW);

  int error = distance - s_base_position;
  int speed_var = s_kp * error + s_kd * (error - s_last_error);
  s_last_error = error;
  if (speed_var > 0) //left turn
  {
    analogWrite(left_motor_f, (max(0, max_speed - speed_var)));
    analogWrite(right_motor_f, max_speed);
  }
  else // right turn
  {
    analogWrite(left_motor_f, max_speed);
    analogWrite(right_motor_f, (max(0, max_speed + speed_var)));
  }
}
