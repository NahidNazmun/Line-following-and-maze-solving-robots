#define trig_f 20
#define echo_f 21
#define trig_r 22
#define echo_r 23
#define front_block 1500
#define base_position_s 1500
#define max_dist_s 3000 //2*base position
#define kp_s .45
#define kd_s 16 

int position = 0;
void setup() 
{
  pinMode(trig_f, OUTPUT);
  pinMode(echo_f, INPUT);
  pinMode(trig_r, OUTPUT);
  pinMode(echo_r, INPUT);  
}

void loop() 
{
  // read front sensor for object
  long front = sonar_f();
  if(front <= front_block)
  {
    long t;
    //break
    do
    {
      //turn left
      t = sonar_r();
    }while(t>base_position_s);

    do
    {
      int position_s = sonar_r();
      // PD using sonar_r
//      position = readline();
    }while(position == 7000);    
  }
}

long sonar_f ()
{
  digitalWrite(trig_f,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_f,LOW);
  long t = pulseIn(echo_f,HIGH,max_dist_s);
  if(!t)
  return max_dist_s;
  return t;
}


long sonar_r ()
{
  digitalWrite(trig_r,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_r,LOW);
  long t = pulseIn(echo_r,HIGH,max_dist_s);
  if(!t)
  return max_dist_s;
  return t;
}
