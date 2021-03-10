/*this fucntion is written only for UNO and not written in a manner so
  that it can be used with any arduino. the input register will need to be
  changed if the pins or the board using is changed.*/

/* this fucntion should work if the port I'm working with is not shared by any other functions*/
#define num_sensors 6
byte s[num_sensors];
byte position = 0;
byte last_position = 0;
byte base_position = (1*10+num_sensors*10)/2;
boolean on_line = true;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  readline();
  for (int i = 0; i < 6; i++)
  {
    Serial.print(s[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}

void readline()
{
  position = 0;
  byte  sum = 0;
  byte t = PIND; // reading into an variable as PIND might change
  t=t>>2; // as the sensors are connected to pin 2 to 7, the values of 0 and 1 pin will create trouble
  if (t)
  {
    on_line = true;
    for (int i = 0; i < num_sensors; i++)
    {
      s[i] = t & (1 << i);
      s[i] = s[i] >> i;
      position += ((i + 1) * 10 * s[i]);
      sum += s[i];
    }
    position = position / sum;
    last_position = position;
  }
  else
  {
    position = last_position;
    on_line = false;
  }
}
