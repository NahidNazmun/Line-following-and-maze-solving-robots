#include <QTRSensors.h>

#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 38

QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int min_c[8] = {960, 1100, 630, 270, 280, 650, 921, 360};
int max_c[8] = {2200, 2300, 2200, 2200, 1800, 2300, 2100, 2200};
int high = 1000;
int pre_value;

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
    sensorValues[i] = 2500 - sensorValues[i];
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

void setup()
{
  //  pinMode(38, OUTPUT);
  //  digitalWrite(38, HIGH);
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
}

void loop()
{
  unsigned int position;
  position = readline();
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position); // comment this line out if you are using raw values
//
  delay(150);
}
