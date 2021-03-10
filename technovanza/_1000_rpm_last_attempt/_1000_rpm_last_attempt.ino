//for readline
/*for readline port c is used
   the value of rx is taken from pin 33, 35, 37, 39, 41, 45, 47, 49
   pin 43 is not used because it's inclueded in port a
   right most sensor is connected to pin 33
*/
unsigned int pre_position;
unsigned int position;
#define sensor_num 8
boolean sensor_value[sensor_num];

/*Motor pin*/
#define l_motor_f 4
#define l_motor_b 3
#define r_motor_f 5
#define r_motor_b 6
#define l_pwm 2
#define r_pwm 7

//PD control
int max_speed = 255;
#define base_position 35
float kp = 2 * max_speed / base_position ;
float kd = 2.1 * kp ;
int last_error = 0;
int last_speed_var = 0;

//this variables are used for breaking the motor
// if program use break then it will set r_break or l_ break as true
boolean r_break = false;
boolean l_break = false;
boolean full_break = false;
boolean line = false;

void setup() {
  pinMode(l_motor_f, OUTPUT);
  pinMode(l_motor_b, OUTPUT);
  pinMode(r_motor_b, OUTPUT);
  pinMode(r_motor_f, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(33, INPUT);
  pinMode(35, INPUT);
  pinMode(37, INPUT);
  pinMode(39, INPUT);
  pinMode(41, INPUT);
  pinMode(45, INPUT);
  pinMode(47, INPUT);
  pinMode(49, INPUT);
  digitalWrite(l_motor_b, LOW);
  digitalWrite(r_motor_b, LOW);
  digitalWrite(l_motor_f, HIGH);
  digitalWrite(r_motor_f, HIGH);
  Serial.begin(9600);
}

void loop() {
  if (full_break)
  {
    digitalWrite(l_motor_f, HIGH);
    digitalWrite(l_motor_b, HIGH);
    digitalWrite(r_motor_f, HIGH);
    digitalWrite(r_motor_b, HIGH);
  }
  max_speed = 255;
  read_whiteline();
  if (!line)
  {
    max_speed = 150;
    //    digitalWrite(l_motor_f, HIGH);
    //    digitalWrite(l_motor_b, HIGH);
    //    digitalWrite(r_motor_f, HIGH);
    //    digitalWrite(r_motor_b, HIGH);
    //    delayMicroseconds(200);
    //    if (position > 35) //left turn
    //    {
    //      digitalWrite(l_motor_b, HIGH);
    //      digitalWrite(l_motor_f, LOW);
    //      digitalWrite(r_motor_b, LOW);
    //      digitalWrite(r_motor_f, HIGH);
    //      analogWrite(l_pwm, 0);
    //      analogWrite(r_pwm, 150);
    //      do{
    //        read_whiteline();
    //      }while(!line);
    //    }
    //    else
    //    {
    //      digitalWrite(r_motor_b, HIGH);
    //      digitalWrite(r_motor_f, LOW);
    //      digitalWrite(l_motor_b, LOW);
    //      digitalWrite(l_motor_f, HIGH);
    //      analogWrite(l_pwm, 150);
    //      digitalWrite(r_pwm, 0);
    //      do{
    //        read_whiteline();
    //      }while(!line);
    //    }
  }
  follow_line();
}

void follow_line()
{
  int error = position - base_position;
  int speed_var = kp * error + kd * (error - last_error);
  last_error = error;
  // if the error is same then it won't change the motor speed
  if (speed_var != last_speed_var)
  {
    if (speed_var > 0) //left turn
    {
      if (speed_var >= 255)
      { //left break
        digitalWrite(l_motor_f, HIGH);
        digitalWrite(l_motor_b, HIGH);
        digitalWrite(l_pwm, HIGH);
        analogWrite(r_pwm, max_speed);
        l_break = true;
      }
      else
      {
        if (l_break)
        {
          digitalWrite(l_motor_f, HIGH);
          digitalWrite(l_motor_b, LOW);
          l_break = false;
        }
        if (r_break)
        {
          digitalWrite(r_motor_f, HIGH);
          digitalWrite(r_motor_b, LOW);
          r_break = false;
        }
        analogWrite(l_pwm, max_speed - speed_var); // for avoiding negative
        analogWrite(r_pwm, max_speed);
      }
    }

    else if (speed_var < 0) //right turn
    {
      if (speed_var <= -255)
      { //right break
        digitalWrite(r_motor_f, HIGH);
        digitalWrite(r_motor_b, HIGH);
        digitalWrite(r_pwm, HIGH);
        analogWrite(l_pwm, max_speed);
        r_break = true;
      }
      else
      {
        if (l_break)
        {
          digitalWrite(l_motor_f, HIGH);
          digitalWrite(l_motor_b, LOW);
          l_break = false;
        }
        if (r_break)
        {
          digitalWrite(r_motor_f, HIGH);
          digitalWrite(r_motor_b, LOW);
          r_break = false;
        }
        analogWrite(r_pwm, max_speed + speed_var); // for avoiding negative
        analogWrite(l_pwm, max_speed);
      }
    }
    //    last_speed_var = speed_var;
  }
}

void read_whiteline()
{
  full_break = false;
  position = 0;
  byte sum = 0;
  line = false;
  port_reading();
  for ( int i = 0; i < 8; i++)
  {
    sensor_value[i] = !(sensor_value[i]);
    position += (i * 10 * sensor_value[i]);
    sum += sensor_value[i];
    if (sensor_value[i]) line = true;
  }
  if (sum == 8) // all on white
    full_break = true;

  if (line)
  {
    position = position / sum;
    pre_position = position;
  }
  else
  {
    position = pre_position;
  }
}

void port_reading()
{
  int c = REG_PIOC_PDSR; // taking the value from port c
  c = c >> 1; // shifting register value( 1 bit right shifting)
  for ( int i = 0 ; i < 5; i++)
  {
    // taking value from pin 33, 35, 37, 39, 41
    sensor_value[i] = c & 1;
    c = c >> 2;
  }
  c = c >> 3;
  for (int i = 0; i < 3; i++)
  {
    // taking value from pin 49, 47, 45
    sensor_value[7 - i] = c & 1;
    c = c >> 2;
  }
}
