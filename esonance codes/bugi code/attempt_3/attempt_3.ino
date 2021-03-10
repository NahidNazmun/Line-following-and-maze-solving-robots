#include <QTRSensors.h>

/*qtrc*/
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 38


int min_c[8] = {710, 715, 310, 130, 145, 395, 620, 190};
int max_c[8] = {2300, 2300, 1500, 700, 730, 2150, 2300, 1100};
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
float kp = 0.2 ;
float kd = 22 ;
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
  //  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  //  {
  //    Serial.print(sensorValues[i]);
  //    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  //  }
  //  Serial.println(position);
  //  delay(150);
  if (position > 6000)
  {
    digitalWrite(l_motor_b, HIGH);
    digitalWrite(r_motor_b, LOW);
    digitalWrite(l_motor_f, LOW);
    digitalWrite(r_motor_f, HIGH);
    analogWrite(l_pwm, 150);
    analogWrite(r_pwm, 150);
    do {
      bluetooth();
      position = readline();
    } while (sensorValues[3] < 1500 && sensorValues[4] < 1500);
  }

  else  if (position < 1000)
  {
    digitalWrite(l_motor_b, LOW);
    digitalWrite(r_motor_b, HIGH);
    digitalWrite(l_motor_f, HIGH);
    digitalWrite(r_motor_f, LOW);
    analogWrite(r_pwm, 150);
    analogWrite(l_pwm, 150);
    do {
      bluetooth();
      position = readline();
    } while (sensorValues[3] < 1500 && sensorValues[4] < 1500);
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
    analogWrite(l_pwm, 0);
    analogWrite(r_pwm, 0);
    char a = Serial1.read();
    Serial1.println("delays");
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

