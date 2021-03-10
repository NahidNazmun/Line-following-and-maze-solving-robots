#include <QTRSensors.h>

//for readline
unsigned int pre_value;
unsigned int position;
#define sensor_num 8
boolean sensor_value[sensor_num];

/*Motor*/
#define l_motor_f 4
#define l_motor_b 3
#define r_motor_f 5
#define r_motor_b 6
#define l_pwm 2
#define r_pwm 7

//PD control
#define max_speed 255
#define base_position 35
float kp = 1 * max_speed / base_position ;
float kd = 2.1* kp ;
int last_error = 0;

void setup() {
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  //  for (byte i = 0; i < sensor_num; i++)
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  pinMode(30, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT);
  Serial.begin(9600);
}

void loop() {
  readline();
  follow_line();
}


void readline()
{
  position = 0;
  byte sum = 0;
  boolean line = false;
  //  int c = REG_PIOD_PDSR; //taking input from pin 25 to 51
  //  c = c >> 1; // if you want to take input from pin 26 then active this line
  for (int i = 0; i < sensor_num; i++)
  {
    sensor_value[i] = digitalRead(25 + i);
    //    sensor_value[i] = c & 1; //taking input from pin (25+i)
    //    c = c >> 1; //shifting register value ( 1 bit right shifting)
    ////    c = c >> 1; // if you want to shift 2 bit active this line
    position += (i * 10 * sensor_value[i]);
    sum += sensor_value[i];
    if (sensor_value[i])
      line = true;
  }
  if (line)
  {
    position = position / sum;
    pre_value = position;
  }
  else
    position = pre_value;
}

void follow_line()
{
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, HIGH);
  int error = position - base_position;
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
