// motor
#define left_motor_f 5 //pwm pins
#define left_motor_b 9
#define right_motor_f 6
#define right_motor_b 10
void setup() {
  pinMode(left_motor_f,OUTPUT);
  pinMode(left_motor_b,OUTPUT);
  pinMode(right_motor_f,OUTPUT);
  pinMode(right_motor_b,OUTPUT);
  // put your setup code here, to run once:

}
int i=0;
void loop() 
{
  analogWrite(right_motor_f,150);
  analogWrite(left_motor_f,150);
  analogWrite(right_motor_b,0);
  analogWrite(left_motor_b,0);
  delay(250);
  analogWrite(right_motor_f,0);
  analogWrite(left_motor_f,200);
  analogWrite(right_motor_b,0);
  analogWrite(left_motor_b,0);
  delay(500);
}

