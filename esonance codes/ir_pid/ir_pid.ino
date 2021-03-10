#include <QTRSensors.h>

//for readline
/*qtrc*/
#define num_sensors 8
#define TIMEOUT 2500
#define EMITTER_PIN 38
int min_rc[8] = {322, 191, 205, 249, 177, 118, 133, 235};
int max_rc[8] = {1425, 830, 874, 1261, 890, 622, 740, 1383};
QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
}, // 0 no sensor= right
num_sensors, TIMEOUT, EMITTER_PIN);
unsigned int sensor_values[num_sensors];

int pre_value = 3500;
#define readline_high  1500
boolean black_line;
unsigned int position;


/*Motor*/
#define l_motor_f 4
#define l_motor_b 3
#define r_motor_f 5
#define r_motor_b 6
#define l_pwm 2
#define r_pwm 7

//PD control
float kp = 0 ;
float kd = 0 ;
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
  while (!Serial1.available());
  char a = Serial1.read();
  if (analogRead(A0) > 500 && analogRead(A1) > 500)
    black_line = false;
  else
    black_line = true;
  
}

void loop() {
  bluetooth();
  position = readline();
//    for (unsigned char i = 0; i < NUM_SENSORS; i++)
//    {
//      Serial.print(sensorValues[i]);
//      Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
//    }
//    Serial.println(position);
//    delay(250);
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

void bluetooth()
{
  if (Serial1.available() > 0)
  {
    analogWrite(l_pwm, 0);
    analogWrite(r_pwm, 0);
    char a = Serial1.read();
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

