void setup() 
{
  Serial.begin(115200);
}

void loop() {
  for(int i=0; i<6; i++)
  {
    Serial.print(analogRead(0));
    Serial.print("\t");
  }
  Serial.println();
  delay(200);
  // put your main code here, to run repeatedly:

}
