void setup()
{
  Serial.begin(9600);
}

void loop()
{
  unsigned long t = micros();
  for (int i = 0; i < 1000; i++)
  {
    int sensorValue1 = analogRead(A0);
    int sensorValue2 = analogRead(A1);
    int sensorValue3 = analogRead(A2);
    int sensorValue4 = analogRead(A3);
    int sensorValue5 = analogRead(A4);
    int sensorValue6 = analogRead(A5);
    int sensorValue7 = analogRead(A6);
    int sensorValue8 = analogRead(A7);
  }
  unsigned long d = micros();
  Serial.println(d-t);
  delay(500);
}
