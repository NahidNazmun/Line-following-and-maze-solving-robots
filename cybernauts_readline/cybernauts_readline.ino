#include <QTRSensors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

#define high 2500
#define low 0
#define b_high 800


// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
unsigned int lastValues[NUM_SENSORS];
unsigned int position;
unsigned int lastposition;

unsigned int readline()
{
  qtrrc.read(sensorValues);
  unsigned int pos = 0;
  int sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++)
  {
//    if (sensorValues[i] < low)
//      sensorValues[i] = 0;
//    else if (sensorValues[i] < high)
//      sensorValues[i] = sensorValues[i] - low;
//    else
//      sensorValues[i] = high - low;
    pos = pos + (i * 1000) * sensorValues[i];
    sum = sum + sensorValues[i];
  }

//  for (int j = 0; j < NUM_SENSORS; j++)
//  {
//    if (sensorValues[j] > b_high)
//      goto label;
//  }
//  pos = lastposition;
//  return pos;
//label:
//  for (int j = 0; j < NUM_SENSORS; j++)
//  {
//    lastValues[NUM_SENSORS] = sensorValues[NUM_SENSORS];
//  }
  pos = pos / sum;
//  lastposition = pos;
  return pos;
}

void setup()
{
  //  pinMode(38, OUTPUT);
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
}

void loop()
{
  position = readline();
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values

  delay(250);
}
