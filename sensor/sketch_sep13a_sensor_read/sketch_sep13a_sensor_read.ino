byte s[6];
byte p[6] = {2, 3, 4, 5, 6, 7}; // pin 7 to 2 is connected with the sensor

void setup()
{
  DDRD = DDRD | B00000000;
  Serial.begin(9600);
}

void loop()
{
  byte t = PIND;
  for (int i = 0; i < 6; i++)
  {
    s[i]= t&(1<<p[i]);
    s[i]=s[i]>>p[i];
  }

  for(int i =0;i<6;i++)
  {
    Serial.print(s[i]);
    Serial.print('\t');
  }
  Serial.println();
}
