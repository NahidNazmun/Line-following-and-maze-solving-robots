boolean black_line;

//for analog read
int right_sensor;
int left_sensor;
#define right_sensor_low 60
#define right_sensor_high 1023
#define left_sensor_high 843
#define left_sensor_low 46

void setup()
{
  Serial.begin(9600);
  // for initializing blackline
  if (analogRead(A0) > 500 && analogRead(A1) > 500)
    black_line = false;
  else
    black_line = true;
}

void loop()
{
  analogread();
  Serial.print(left_sensor);
  Serial.print('\t');
  Serial.print(right_sensor);
  Serial.println();
  delay(100);
}
void analogread()
{
  right_sensor = analogRead(A0); //right
  left_sensor = analogRead(A1);
  if (!black_line)
  {
    right_sensor = map(right_sensor, 0, 1023, 1023, 0);
    left_sensor = map(left_sensor, 0, 1023, 1023, 0);
  }
}
