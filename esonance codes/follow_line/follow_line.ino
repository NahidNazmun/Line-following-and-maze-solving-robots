#include <QTRSensors.h>

#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 38

QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int min_c[8] = {619, 565, 378, 230, 243, 431, 593, 257};
int max_c[8] = {2500, 1990, 1352, 784, 825, 1488, 2018, 1258};
#define high 1500
#define turn_high 2000
#define black_line_low 1000

int pre_value;

//PD control
#define kp .3
#define kd 4
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
  Serial1.begin(9600);

  while (!Serial1.available());
}

void loop()
{
  bluetooth();
  position = readline();
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
  last_error = error;
  int speed_var = kp * error + kd * (error - last_error);
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

void bluetooth()
{
  if (Serial1.available() > 0)
  {
    analogWrite(l_pwm, 0);
    analogWrite(r_pwm, 0);
    char a = Serial1.read();
    Serial1.println("give kp kd");
    //    while (!Serial1.available());
    //    kp = Serial1.parseFloat();
    //    Serial.print(kp);
    //    while (!Serial1.available());
    //    kd = Serial1.parseFloat();
    //    Serial.print(kd);
    while (!Serial1.available());
    Serial1.read();
    delay(500);
  }
}
