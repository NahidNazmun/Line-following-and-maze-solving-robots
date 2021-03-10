int s0 = 7;
int s1 = 6;
int s2 = 5;
int s3 = 4;
int out = 3;

void setup()
{
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  Serial.begin(9600);

  /*
     L L POWER DOWN
     L H 2%
     H L 20%
     H H 100%
  */

  digitalWrite(s0, LOW);
  digitalWrite(s1, HIGH);


}

void loop()
{
  digitalWrite(s2, HIGH); //CLEAR
  digitalWrite(s3, LOW);
  unsigned int clear = pulseIn(out, HIGH);


  digitalWrite(s2, LOW); //RED
  digitalWrite(s3, LOW);
  unsigned int red = pulseIn(out, HIGH);


  digitalWrite(s2, HIGH); //BLUE
  digitalWrite(s3, HIGH);
  unsigned int blue = pulseIn(out, HIGH);

  digitalWrite(s2, HIGH); //GREEN
  digitalWrite(s3, HIGH);
  unsigned int green = pulseIn(out, HIGH);

  Serial.print(clear);
  Serial.print('\t');

  Serial.print(red);
  Serial.print('\t');

  Serial.print(green);
  Serial.print('\t');

  Serial.println(blue);

  delay(500);
}
