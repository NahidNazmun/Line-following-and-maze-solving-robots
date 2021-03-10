#include <digitalWriteFast.h>
#include <QTRSensors.h>
//#include <SoftwareSerial.h>


/*qtrc*/
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN QTR_NO_EMITTER_PIN

int min_c[8] = {550, 443, 354, 265, 280, 354, 503, 282};
int max_c[8] = {1300, 1010, 843, 625, 636, 800, 1144, 1026};
int high = 1000;
int pre_value;

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

//motor
#define l_motor_f 10
#define l_motor_b 11
#define r_motor_f 12
#define r_motor_b 13
#define l_pwm 6
#define r_pwm 7



//PD control
#define kp .3
#define kd 2.95
#define max_speed 100
#define a_speed 70
#define base_position 3500
int last_error = 0;
int b = 0;
int d = 100;

#define b_high 2000
#define b_low 500


unsigned int position;

boolean j = false;

void setup()
{
  //  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(38, OUTPUT);
  digitalWrite(38, HIGH);
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
}

void loop()
{
  position = readline();
  bluetooth();
  if (sensorValues[0] > b_high && sensorValues[1] > b_high && sensorValues[2] > b_high && sensorValues[3] > b_high && sensorValues[7] < b_high)
  {
    analogWrite(l_pwm, 0);
    analogWrite(r_pwm, 0);
    delay(b);
    //    do
    //    {
    //      motor(l, FORWARD, 0);
    //      motor(r, BACKWARD, 0);
    //      position = readline();
    //    } while (sensorValues[0] > b_low);
    //    motor(l, BACKWARD, max_speed);
    //    motor(r, FORWARD, max_speed);
    digitalWrite(l_motor_f, LOW);
    digitalWrite(l_motor_b, HIGH);
    digitalWrite(r_motor_f, HIGH);
    digitalWrite(r_motor_b, LOW);
    analogWrite(l_pwm, max_speed);
    analogWrite(r_pwm, max_speed);
    delay(d);
    do
    {
      bluetooth();
      position = readline();
    } while (sensorValues[3] < b_high && sensorValues[4] < b_high);
  }
  else
    follow_line(position);
}

void bluetooth()
{
  Serial1.println("running");
  if (Serial1.available() > 0)
  {
    analogWrite( l_pwm, 0);
    analogWrite(r_pwm, 0);
    char a = Serial1.read();
    //    if (a == 'p' || a == 'P')
    //    {
    Serial1.println("stop");
    while (!Serial1.available());
    Serial1.read();
    delay(300);
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



void follow_line (unsigned int pos)
{
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_b, LOW);
  int error = pos - base_position;
  last_error = error;
  int speed_var = kp * error + kd * (error - last_error);
  int var_speed;
  if (speed_var < 0) //left turn
  {
    var_speed = max(0, max_speed + speed_var);
    digitalWrite(r_motor_f, HIGH);
    digitalWrite(l_motor_f, HIGH);
    analogWrite(l_pwm, var_speed);
    analogWrite(r_pwm, max_speed);

  }
  else // right turn
  {
    var_speed = max(0, max_speed - speed_var);
    digitalWrite(r_motor_f, HIGH);
    digitalWrite(l_motor_f, HIGH);
    analogWrite(r_pwm, var_speed);
    analogWrite(l_pwm, max_speed);
  }
}

