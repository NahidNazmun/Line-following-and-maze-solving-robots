// for 100 kp .3 kd 2.95
// or kp .2 and kd 1.95




#include <digitalWriteFast.h>
#include <QTRSensors.h>
//#include <SoftwareSerial.h>


/*qtrc*/
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN QTR_NO_EMITTER_PIN

int min_c[8] = {91,359, 209,61,61,120,254,343};
int max_c[8] = {535, 1508, 860, 296, 296, 565, 1044, 1435};
int high = 1500;
int pre_value;

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];


/*motor*/
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4


//PD control
float kp = 0;
float kd = 0;
#define max_speed 100
#define base_position 3500
int last_error = 0;
int var_speed;

unsigned int position;

boolean j = false;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(38, OUTPUT);
  digitalWrite(38, HIGH);
}

void loop()
{
  if (Serial1.available() > 0)
  {
    motor(1, FORWARD, 0);
    motor(2, BACKWARD, 0);
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
    var_speed = max(0, max_speed + speed_var);
    motor(1, FORWARD, var_speed);
    motor(2, BACKWARD, max_speed);
  }
  else // right turn
  {
    var_speed = max(0, max_speed - speed_var);
    motor(1, FORWARD, max_speed);
    motor(2, BACKWARD, var_speed);
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

/* for motor*/

void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {
    switch (nMotor)
    {
      case 1:
        motorA   = MOTOR1_A;
        motorB   = MOTOR1_B;
        break;
      case 2:
        motorA   = MOTOR2_A;
        motorB   = MOTOR2_B;
        break;
    }

    switch (command)
    {
      case FORWARD:
        motor_output (motorA, HIGH, speed);
        motor_output (motorB, LOW, -1);     // -1: no PWM set
        break;
      case BACKWARD:
        motor_output (motorA, LOW, speed);
        motor_output (motorB, HIGH, -1);    // -1: no PWM set
        break;
      case BRAKE:
        motor_output (motorA, LOW, 255); // 255: fully on.
        motor_output (motorB, LOW, -1);  // -1: no PWM set
        break;
      case RELEASE:
        motor_output (motorA, LOW, 0);  // 0: output floating.
        motor_output (motorB, LOW, -1); // -1: no PWM set
        break;
      default:
        break;
    }
  }
}

void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
    case MOTOR1_A:
    case MOTOR1_B:
      motorPWM = MOTOR1_PWM;
      break;
    case MOTOR2_A:
    case MOTOR2_B:
      motorPWM = MOTOR2_PWM;
      break;
    default:
      speed = -3333;
      break;
  }

  if (speed != -3333)
  {
    shiftWrite(output, high_low);

    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPWM, speed);
    }
  }
}

void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;
  if (!shift_register_initialized)
  {
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);

    digitalWrite(MOTORENABLE, LOW);

    latch_copy = 0;

    shift_register_initialized = true;
  }

  bitWrite(latch_copy, output, high_low);

  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  digitalWrite(MOTORLATCH, HIGH);
  digitalWrite(MOTORLATCH, LOW);
}

