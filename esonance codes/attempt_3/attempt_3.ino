#include <QTRSensors.h>

/*qtrc*/
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 38


int min_c[8] = {380, 380, 200, 100, 130, 370, 600, 520};
int max_c[8] = {1800, 1400, 900, 550, 650, 1600, 2200, 1400};
#define readline_high  1500

int pre_value;

/*Motor*/
#define l_motor_f 4
#define l_motor_b 3
#define r_motor_f 5
#define r_motor_b 6
#define l_pwm 2
#define r_pwm 7
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

unsigned int position;

//PD control
#define kp  .2
#define kd  2.2
#define max_speed 255
#define base_position 3500
int last_error = 0;

boolean black_line;
#define high 2000
#define low 500
#define turn_high 1500
#define analog_high  700
#define analog_low  500
#define right_sensor_high 900
#define right_sensor_low 300
#define left_sensor_high 900
#define left_sensor_low 300
int right_sensor; // A1
int left_sensor; // A0



void setup() {
  black_line = true;
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  right_sensor = analogRead(A1);
  left_sensor = analogRead(A0);
  Serial1.println("delays");
  while (!Serial1.available());
  Serial1.read();
  delay(500);
  
//  if ( right_sensor < 500 && left_sensor < 500)
//    black_line = true;
//  else
//    black_line = false;
}

void loop() {
start:
  bluetooth();
  position = readline();
  analogread();

  if (right_sensor > analog_high && (sensorValues[3] > high || sensorValues[4] > high))
  {

    analogWrite(l_pwm, 200);
    analogWrite(r_pwm, 200);
    do {
      bluetooth();
      analogread();
      if (left_sensor > analog_high)
      {
tsection:
        unsigned int tm = millis();; //track switch
        do {
          analogread();
          if ((millis() - tm) > 150)
          {
            black_line = !(black_line);
            Serial.println("SWITCH");
            goto start;
          }
        } while (left_sensor > analog_low);
      }
      analogread();
    } while (right_sensor > analog_low);
    do {
      bluetooth();
      position = readline();
    } while (sensorValues[0] < turn_high || sensorValues[1] < turn_high);
    right_turn();
  }

  else if (left_sensor > analog_high && (sensorValues[3] > high || sensorValues[4] > high))
  {
    analogWrite(l_pwm, 200);
    analogWrite(r_pwm, 200);
    do {
      bluetooth();
      analogread();
      if (right_sensor > analog_high)
        goto tsection;
    } while (left_sensor > analog_low);
    do {
      bluetooth();
      position = readline();
    } while (sensorValues[7] < turn_high || sensorValues[6] < turn_high);
    left_turn();
  }


  else
    follow_line(position);
}




void follow_line(unsigned int pos)
{
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, HIGH);
  int error = pos - base_position;
  int speed_var = kp * error + kd * (error - last_error);
  last_error = error;
  if (speed_var > 0) //left turn
  {
    analogWrite(l_pwm, max(0, max_speed - speed_var));
    analogWrite(r_pwm, max_speed);
  }
  else //right turn
  {
    analogWrite(r_pwm, max(0, max_speed + speed_var));
    analogWrite(l_pwm, max_speed);
  }

}



void right_turn()
{
  digitalWrite(r_motor_b, HIGH);
  digitalWrite(r_motor_f, LOW);
  digitalWrite(l_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  delay(300);
  do {
    bluetooth();
    position = readline();
  } while (sensorValues[3] < high && sensorValues[4] < high);
}



void left_turn()
{
  digitalWrite(l_motor_b, HIGH);
  digitalWrite(l_motor_f, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(r_motor_f, HIGH);
  delay(300);
  do {
    bluetooth();
    position = readline();
  } while (sensorValues[3] < high && sensorValues[4] < high);
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
    if (black_line)
    {
      sensorValues[i] = map(sensorValues[i], min_c[i], max_c[i], 0, 2500);
    }
    else
    {
      sensorValues[i] = map(sensorValues[i], min_c[i], max_c[i], 2500, 0);
    }
    if (sensorValues[i] > readline_high)
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

void analogread()
{
  right_sensor = analogRead(A0); //right
  left_sensor = analogRead(A1);
  right_sensor = (right_sensor, right_sensor_low, right_sensor_high);
  left_sensor = (left_sensor, left_sensor_low, left_sensor_high);
  if (black_line)
  {
    right_sensor = map(right_sensor, right_sensor_low, right_sensor_high, 0, 1000);
    left_sensor = map(left_sensor, left_sensor_low, left_sensor_high, 0, 1000);
  }
  else
  {
    right_sensor = map(right_sensor, right_sensor_low, right_sensor_high, 1000, 0);
    left_sensor = map(left_sensor, left_sensor_low, left_sensor_high, 1000, 0);
  }
}

void bluetooth()
{
  if (Serial1.available() > 0)
  {
    stop();
    char a = Serial1.read();
    Serial1.println("delays");
    while (!Serial1.available());
    Serial1.read();
    delay(500);
  }
}

void stop()
{
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, LOW);
  digitalWrite(r_motor_f, LOW);
  analogWrite(l_pwm, 0);
  analogWrite(r_pwm, 0);
}

