/* Program to read QTRsensors values*/

//for white line on black board

#include <QTRSensors.h>

QTRSensorsRC qtr((unsigned char[]) {A0,12,A1,A2,A3,A4,11,A5}, 8, 2500);
 
void setup()
{ 
  Serial.begin(9600);
  for(int i=0; i<20 ; i++)
  { qtr.calibrate();
    delay(100);
  }
}

 
void loop()
{
  unsigned int sensors[8];
  
  int position = qtr.readLine(sensors);
  

  for (unsigned char i = 0; i < 8; i++)
    {
      
      Serial.print(sensors[i]);
      Serial.print('\t');
    }
    Serial.println();
    Serial.println(position);
}
