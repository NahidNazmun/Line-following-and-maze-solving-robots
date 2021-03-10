void setup()
{
  pinMode(7,INPUT);
  Serial.begin(9600);
}

void loop()
{
  byte s = PIND;
  byte t = s&(1<<7); // the input is in the pin no. 7
  t=t>>7;
  Serial.println(t);
}
