#include <digitalWriteFast.h>
#include <QTRSensors.h>

//bluetooth
#include <SoftwareSerial.h>
int BL_Tx = 16;
int BL_Rx = 17;

SoftwareSerial BL(BL_Tx, BL_Rx);

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

#define high 2500
#define low 200
#define b_high 500

// motor pins
#define left_motor 4
#define right_motor 3

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
unsigned int lastValues[NUM_SENSORS];
unsigned int lastposition;

//PD control
#define kp .1
#define kd 2
#define max_speed 100
#define base_position 3500
int last_error = 0;

unsigned int position;

void setup()
{
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(14, HIGH);
  digitalWrite(15, LOW);
  pinMode(left_motor, OUTPUT);
  pinMode(right_motor, OUTPUT);
  Serial.begin(9600);
  BL.begin(9600);

}

void loop()
{
  position = readline();

  float graph = 5.0 * position / 7000;
  BL.println(graph);

  int error = position - base_position;
  last_error = error;
  int speed_var = kp * error + kd * (error - last_error);

  if (speed_var > 0) //left turn
  {
    analogWrite(left_motor, (max(0, max_speed - speed_var)));
    analogWrite(right_motor, max_speed);
  }
  else // right turn
  {
    analogWrite(left_motor, max_speed);
    analogWrite(right_motor, (max(0, max_speed + speed_var)));
  }
}

unsigned int readline()
{
  qtrrc.read(sensorValues);
  unsigned int pos = 0;
  int sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] < low)
      sensorValues[i] = 0;
    else if (sensorValues[i] < high)
      sensorValues[i] = sensorValues[i] - low;
    else
      sensorValues[i] = high - low;
    pos = pos + (i * 1000) * sensorValues[i];
    sum = sum + sensorValues[i];
  }

  for (int j = 0; j < NUM_SENSORS; j++)
  {
    if (sensorValues[j] > b_high )
      goto label;
  }
  pos = lastposition;
  return pos;
label:
  for (int j = 0; j < NUM_SENSORS; j++)
  {
    lastValues[NUM_SENSORS] = sensorValues[NUM_SENSORS];
  }
  pos = pos / sum;
  lastposition = pos;
  return pos;
}
