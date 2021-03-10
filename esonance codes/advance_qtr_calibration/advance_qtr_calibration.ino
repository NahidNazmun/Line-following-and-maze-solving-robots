#include <QTRSensors.h>

//for readline
/*qtrc*/
#define num_sensors 8
#define TIMEOUT 2500
#define EMITTER_PIN 38
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
},
num_sensors, TIMEOUT, EMITTER_PIN);
unsigned int sensor_values[num_sensors];

int max_rc[8][5][2] = {{{0, 0} , {0, 0} , {0, 0} , {0, 0} , {0, 0}}, {{0, 0} , {0, 0} , {0, 0} , {0, 0} , {0, 0}},
  {{0, 0} , {0, 0} , {0, 0} , {0, 0} , {0, 0}}, {{0, 0} , {0, 0} , {0, 0} , {0, 0} , {0, 0}}, {{0, 0} , {0, 0} , {0, 0} , {0, 0} , {0, 0}}, {{0, 0} , {0, 0} , {0, 0} , {0, 0} , {0, 0}},
  {{0, 0} , {0, 0} , {0, 0} , {0, 0} , {0, 0}}, {{0, 0} , {0, 0} , {0, 0} , {0, 0} , {0, 0}}
};


int min_rc[8][5][2] = {{{2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}}, {{2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}}, {{2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}},
  {{2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}}, {{2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}}, {{2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}},
  {{2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}}, {{2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}, {2500, 0}}
};

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  delay(500);
  digitalWrite(13, HIGH);
  for ( int i = 0 ; i < 4000; i++)
  {
    qtrrc.read(sensor_values);
    for ( byte x = 0; x < 8; x++)
    {
      boolean mx = true;
      boolean mn = true;
      for ( byte y = 0; y < 5; y++)
      {
        //for maximum values
        if (sensor_values[x] == max_rc[x][y][0])
        {
          max_rc[x][y][1] += 1;
          mx = false;
        }

        else if ((sensor_values[x] > max_rc[x][y][0]) && (mx))
        {
          for (byte z = y; z < 4; z++)
          {
            max_rc[x][z + 1][0] =  max_rc[x][z][0];
            max_rc[x][z + 1][1] =  max_rc[x][z][1];
          }
          max_rc[x][y][0] = sensor_values[x];
          max_rc[x][y][1] = 1;
          mx = false;
        }

        //for minimum values
        if (sensor_values[x] == min_rc[x][y][0])
        {
          min_rc[x][y][1] += 1;
          mn=false;
        }

        else if ((sensor_values[x] < min_rc[x][y][0]) && (mn))
        {
          for (byte z = y; z < 4; z++)
          {
            min_rc[x][z + 1][0] =  min_rc[x][z][0];
            min_rc[x][z + 1][1] =  min_rc[x][z][1];
          }
          min_rc[x][y][0] = sensor_values[x];
          min_rc[x][y][1] = 1;
          mn=false;
        }
      }
    }
  }
  digitalWrite(13, LOW);
  //printing minimum values
  Serial.println("Minimum Values");
  for ( byte j = 0; j < 8; j++)
  {
    Serial.print(j);
    Serial.print(":\t");
    for (byte i = 0; i < 5; i++)
    {
      Serial.print(min_rc[j][i][0]);
      Serial.print("(");
      Serial.print(min_rc[j][i][1]);
      Serial.print(")\t\t");
    }
    Serial.println();
  }
  Serial.println();

  //printing maximum values
  Serial.println("maximum Values");
  for ( byte j = 0; j < 8; j++)
  {
    Serial.print(j);
    Serial.print(":\t");
    for (byte i = 0; i < 5; i++)
    {
      Serial.print(max_rc[j][i][0]);
      Serial.print("(");
      Serial.print(max_rc[j][i][1]);
      Serial.print(")\t\t");
    }
    Serial.println();
  }
  Serial.println();
  Serial.println();

  while (1);

}

