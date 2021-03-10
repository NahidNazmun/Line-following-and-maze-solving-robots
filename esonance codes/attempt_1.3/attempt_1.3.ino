#include <QTRSensors.h>

/*qtrc*/
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 38


int min_c[8] = {950, 790, 700, 650, 670, 800, 1050, 1150};
int max_c[8] = {2300, 2300, 2200, 2300, 2200, 2300, 2300, 2300};
#define high  1500

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
float kp = .1 ;
float kd = 1.3 ;
#define max_speed 255
#define base_position 3500
int last_error = 0;

void setup() {
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);

}

void loop() {
  bluetooth();
  position = readline();
  int a = analogRead(A0);
  int b = analogRead(A1);

  if (a > 700 && (sensorValues[3] > 2000 || sensorValues[4] > 2000))
  {
    analogWrite(l_pwm, 200);
    analogWrite(r_pwm, 200);
    do {
      bluetooth();
      a = analogRead(A0);
    } while (a > 500);
    do {
      bluetooth();
      position = readline();
    } while (sensorValues[0] < 1500 || sensorValues[1] < 1500);
    right_turn();
  }

  else if (b > 700 && (sensorValues[3] > 2000 || sensorValues[4] > 2000))
  {
    boolean rg = false;
    analogWrite(l_pwm, 200);
    analogWrite(r_pwm, 200);
    do {
      bluetooth();
      a = analogRead(A0);
      b = analogRead(A1);
      if (a > 700)
        rg = true;
    } while (b > 500);
    if (rg)
    {
      do {
        bluetooth();
        position = readline();
      } while (sensorValues[0] < 1500 || sensorValues[1] < 1500);
      right_turn();
    }
    else
    {
      do {
        bluetooth();
        position = readline();
      } while (sensorValues[7] < 1500 || sensorValues[6] < 1500);
      left_turn();
    }
  }

  else if ((sensorValues[1] > 2000 || sensorValues[2] > 2000) && (sensorValues[3] < 500 || sensorValues[4] < 500) && (sensorValues[5] > 2000 || sensorValues[6] > 2000))
  {
    analogWrite(l_pwm, 150);
    analogWrite(r_pwm, 150);
    do {
      bluetooth();
      position = readline();
      if (sensorValues[3] > 2000)
      {
        analogWrite(l_pwm, 0);
        analogWrite(r_pwm, 100);
        do {
          position = readline();
        } while (sensorValues[3]  > 500);
        analogWrite(l_pwm, 150);
        analogWrite(r_pwm, 150);
      }
    } while ((sensorValues[1] > 500 || sensorValues[0] > 500)  && (sensorValues[7] > 500 || sensorValues[6] > 500));
    unsigned int sc = millis();
    while ((millis() - sc) < 300)
    {
      follow_line(position);
      position = readline();
    }
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
  last_error = error;
  int speed_var = kp * error + kd * (error - last_error);
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
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, HIGH);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, LOW);
  delay(300);
  do {
    bluetooth();
    position = readline();
  } while (sensorValues[3] < 2000 && sensorValues[4] < 2000);
}

void left_turn()
{
  digitalWrite(l_motor_b, HIGH);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, LOW);
  digitalWrite(r_motor_f, HIGH);
  delay(300);
  do {
    bluetooth();
    position = readline();
  } while (sensorValues[3] < 2000 && sensorValues[4] < 2000);
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

