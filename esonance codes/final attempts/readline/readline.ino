#include <QTRSensors.h>

// cybernauts values
//int min_rc[8] = {289, 190, 189, 371, 165, 147, 146, 280};
//int max_rc[8] = {1864, 1277, 1251, 2124, 1058, 981, 956, 1696

//for readline
/*qtrc*/
#define num_sensors 8
#define TIMEOUT 2500
#define EMITTER_PIN 38
int min_rc[8] = {74, 268, 187, 83, 83, 175, 266, 300};
int max_rc[8] = {592, 993, 718, 516, 516, 765, 941, 1113};
QTRSensorsRC qtrrc((unsigned char[]) {
  36, 34, 32, 30, 28, 26, 24, 22
}, // 0 no sensor= right
num_sensors, TIMEOUT, EMITTER_PIN);
unsigned int sensor_values[num_sensors];

int pre_value = 3500;
#define readline_high  1500
boolean black_line;
unsigned int position;


//for analog read
int right_sensor;
int left_sensor;
#define right_sensor_low 200
#define right_sensor_high 800
#define left_sensor_high 700
#define left_sensor_low 200

void setup() {
  Serial.begin(9600);
  if (analogRead(A0) > 500 && analogRead(A1) > 500)
    black_line = false;
  else
    black_line = true;
}

void loop() {
  analogread();
  position = readline();
  //  Serial.print(left_sensor);
  //  Serial.print('\t');
  for (int i = (num_sensors - 1); i >= 0 ; i--)
  {
    Serial.print(sensor_values[i]);
    Serial.print('\t');
  }
  //  Serial.print(right_sensor);
  //  Serial.print("\t\t");
  Serial.println(position);
//  delay(150);
}


unsigned int readline()
{
  unsigned int pos = 0;
  int sum = 0;
  boolean a = false; //for sensing line
  qtrrc.read(sensor_values);
  for ( int i = 0; i < num_sensors; i++)
  {
    sensor_values[i] = constrain(sensor_values[i], min_rc[i], max_rc[i]);
    if (black_line)
    {
      sensor_values[i] = map(sensor_values[i], min_rc[i], max_rc[i], 0, 2500);
    }
    else
    {
      sensor_values[i] = map(sensor_values[i], min_rc[i], max_rc[i], 2500, 0);
    }
    if (sensor_values[i] > readline_high)
      a = true; //found line
    pos += (i * 1000 * sensor_values[i]);
    sum += sensor_values[i];
  }
  if (a)
  {
    pos = pos / sum;
    pre_value = pos;
  }

  else
  {
    if (pre_value > 6000)
      pos = (num_sensors - 1) * 1000;
    else if (pre_value < 1000)
      pos = 0;
    else
      pos = pre_value;
  }
  return pos;
}

void analogread()
{
  right_sensor = analogRead(A0); //right
  left_sensor = analogRead(A1);
  right_sensor = constrain(right_sensor, right_sensor_low, right_sensor_high);
  left_sensor = constrain(left_sensor, left_sensor_low, left_sensor_high);
  if (black_line)
  {
    right_sensor = map(right_sensor, right_sensor_low, right_sensor_high, 0, 2500);
    left_sensor = map(left_sensor, left_sensor_low, left_sensor_high, 0, 2500);
  }
  else
  {
    right_sensor = map(right_sensor, right_sensor_low, right_sensor_high, 2500, 0);
    left_sensor = map(left_sensor, left_sensor_low, left_sensor_high, 2500, 0);
  }
}
