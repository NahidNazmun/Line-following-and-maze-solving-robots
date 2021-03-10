void setup()
{
  Serial.begin(9600);
}

void loop()
{
  unsigned int t = micros();
  for (unsigned int i = 0; i < 10000; i++)
  {
    byte s = PIND;
    s = s >> 2;
  }
  Serial.println(micros() - t);
  while (1);
}
