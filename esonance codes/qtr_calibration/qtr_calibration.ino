#include <QTRSensors.h>

//for readline
/*qtrc*/
#define num_sensors 8
#define TIMEOUT 2500
#define EMITTER_PIN 38
// 0 no sensor= right
QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
},
num_sensors, TIMEOUT, EMITTER_PIN);
unsigned int sensor_values[num_sensors];

int max_rc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int min_rc[8] = { 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
int diff_rc[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//follow line
/*Motor*/
#define l_motor_f 4
#define l_motor_b 3
#define r_motor_f 5
#define r_motor_b 6
#define l_pwm 2
#define r_pwm 7

void setup()
{
//  pinMode(l_motor_f, OUTPUT);
//  pinMode(l_motor_b, OUTPUT);
//  pinMode(r_motor_b, OUTPUT);
//  pinMode(r_motor_f, OUTPUT);
//  pinMode(l_pwm, OUTPUT);
//  pinMode(r_pwm, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  //  delay(1000);
  //  digitalWrite(l_motor_b, HIGH);
  //  digitalWrite(l_motor_f, LOW);
  //  digitalWrite(r_motor_b, LOW);
  //  digitalWrite(r_motor_f, HIGH);
  //  analogWrite(l_pwm, 150);
  //  analogWrite(r_pwm, 150);
  //  analogWrite(13, 100);
  Serial.println("start");
  Serial.println();
  delay(500);
  for ( int i = 0 ; i < 4000; i++)
  {
    qtrrc.read(sensor_values);
    for ( int j = 0; j < 8; j++)
    {
      if (sensor_values[j] > max_rc[j])
      {
        max_rc[j] = sensor_values[j];
      }
      if (sensor_values[j] < min_rc[j])
      {
        min_rc[j] = sensor_values[j];
      }
    }
  }
  for (int i = 0; i < 8; i++)
  {
    diff_rc[i] = (max_rc[i] - min_rc[i]) / 20; // to take the 5% value
    max_rc[i] = max_rc[i] - diff_rc[i];
    min_rc[i] = min_rc[i] + diff_rc[i];
  }
  Serial.println("LOW VALUES:");
  for ( int j = 0; j < 8; j++)
  {
    Serial.print(min_rc[j]);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println("HIGH VALUES:");

  for ( int j = 0; j < 8; j++)
  {
    Serial.print(max_rc[j] - ((max_rc[j] - min_rc[j]) * 5) / 100);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println();
  while (1);
}

