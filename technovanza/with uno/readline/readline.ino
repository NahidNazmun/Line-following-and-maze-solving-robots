//for readline
/*for readline port c & d is used
   the value of rx is taken from pin a0, a1, a2, a3, a4, a5, d4, d2
   right most sensor is connected to pin a0
*/
unsigned int pre_position;
unsigned int position;
#define sensor_num 8
boolean sensor_value[sensor_num];
boolean blackline;

/*motor pin*/
#define r_motor_f 13
#define r_motor_b 12
#define l_motor_b 11
#define l_motor_f 8
#define r_pwm 10
#define l_pwm 9

//PD control
#define max_speed 255
#define base_position 35
float kp = 2 * max_speed / base_position ;
float kd = 2 * kp ;
int last_error = 0;

int last_speed_var = 0;

//this variables are used for breaking the motor
// if program use break then it will set r_break or l_ break as true
boolean r_break = false;
boolean l_break = false;

void setup() {
  blackline = false;
  
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(2, INPUT);
  pinMode(4, INPUT);
//  digitalWrite(l_motor_b, LOW);
//  digitalWrite(r_motor_b, LOW);
//  digitalWrite(l_motor_f, HIGH);
//  digitalWrite(r_motor_f, HIGH);
  Serial.begin(9600);

}

void loop() {
  readline();
  for(byte i=0; i<8; i++)
  {
    Serial.print(sensor_value[7-i]);
    Serial.print('\t');
  }
  Serial.println();
}

void readline()
{
  position = 0;
  byte sum = 0;
  boolean line = false;
  port_reading();
  for ( int i = 0; i < 8; i++)
  {
    if (!blackline)
      sensor_value[i] = !(sensor_value[i]);
    position += (i * 10 * sensor_value[i]);
    sum += sensor_value[i];
    if (sensor_value[i])
      line = true;
  }
  if (line)
  {
    position = position / sum;
    pre_position = position;
  }
  else
    position = pre_position;
}

void port_reading()
{
  int c = PINC;
  for (byte i = 0; i < 6; i++)
  {
    sensor_value[i] = c & 1;
    c = c >> 1;
  }
  c = PIND;
  for (byte i = 0; i < 2; i++)
  {
    c = c >> 2;
    sensor_value[7 - i] = c & 1;
  }
}

