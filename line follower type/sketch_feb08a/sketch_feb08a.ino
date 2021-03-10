#include<QTRSensors.h>

#define num_sensors 6
#define sample_rate 3
#define emitter_pin QTR_NO_EMITTER_PIN

QTRSensorsAnalog qtra ((unsigned char[]) {  0, 1, 2, 3, 4, 5}, num_sensors, sample_rate, emitter_pin);
unsigned int sensor_values[num_sensors];// 0 = leftmost 5 = rightmost
unsigned int position = 2500;

unsigned long sec1 = 0;
unsigned long sec2 = 0;
unsigned long diff = 0;

#define left_motor_f 5 //pwm pins
#define left_motor_b 9
#define right_motor_f 6
#define right_motor_b 10
#define led_pin 13
void setup()
{
  pinMode(left_motor_f, OUTPUT);
  pinMode(left_motor_b, OUTPUT);
  pinMode(right_motor_f, OUTPUT);
  pinMode(right_motor_b, OUTPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  Serial.begin(115200);
  qtra.calibrate();
  qtra.calibratedMinimumOn[0] = 505;
  qtra.calibratedMinimumOn[1] = 332;
  qtra.calibratedMinimumOn[2] = 281;
  qtra.calibratedMinimumOn[3] = 283;
  qtra.calibratedMinimumOn[4] = 299;
  qtra.calibratedMinimumOn[5] = 286;

  qtra.calibratedMaximumOn[0] = 1023;
  qtra.calibratedMaximumOn[1] = 1023;
  qtra.calibratedMaximumOn[2] = 1023;
  qtra.calibratedMaximumOn[3] = 1023;
  qtra.calibratedMaximumOn[4] = 1023;
  qtra.calibratedMaximumOn[5] = 1023;
  delay(1000);
  digitalWrite(led_pin, LOW);
}

void loop()
{
  digitalWrite(left_motor_b, 0);
  digitalWrite(right_motor_b, 0);
  analogWrite(left_motor_f, 180);
  analogWrite(right_motor_f, 180);
  position = qtra.readLine(sensor_values);

  if (sensor_values[0] > 750 || sensor_values[5] > 750)
  {
    delay(30);
//    sec1 = micros();
//    while (sensor_values[1] > 750 && sensor_values[2] > 750 && sensor_values[3] > 750 && sensor_values[4] > 750)
//    { 
//      position = qtra.readLine(sensor_values);
//    }
//    sec2 = micros();
//    Serial.print("difference: ");
//    diff = sec2 - sec1;
//    Serial.println(diff);
    while (1)
    {
      digitalWrite(left_motor_f, LOW);
      digitalWrite(right_motor_f, LOW);
    }
  }
}
