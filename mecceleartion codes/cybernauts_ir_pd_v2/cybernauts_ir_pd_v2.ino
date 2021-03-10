#include <QTRSensors.h>

/*qtrc*/
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 38


int min_c[8] = {91, 359, 209, 61, 61, 120, 254, 343};
int max_c[8] = {535, 1508, 860, 296, 296, 565, 1044, 1435};
int high = 1500;
int pre_value;

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

//PD control
float kp = 0;
float kd = 0;
#define max_speed 255
#define base_position 3500
int last_error = 0;
int var_speed;

unsigned int position;

/*Motor*/
#define l_motor_f 9
#define l_motor_b 8
#define r_motor_f 10
#define r_motor_b 11
#define l_pwm 7
#define r_pwm 12

boolean j = false;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, HIGH);
}

void loop()
{
  if (Serial1.available() > 0)
  {
    analogWrite(l_pwm, 0);
    analogWrite(r_pwm, 0);
    char a = Serial1.read();
    if (a == 'p' || a == 'P')
    {
      Serial1.println("delays");
      while (!Serial1.available());
      kp = Serial1.parseFloat();
      Serial.print(kp);
      while (!Serial1.available());
      kd = Serial1.parseFloat();
      Serial.print(kd);
      while (!Serial1.available());
      Serial1.read();
      delay(500);
    }
  }

  position = readline();
  int error = position - base_position;
  last_error = error;
  int speed_var = kp * error + kd * (error - last_error);
  Serial1.print(error);
  Serial1.print("   ");
  Serial1.println();
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
