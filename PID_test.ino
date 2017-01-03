/* Program to test PID */

#include <QTRSensors.h>

#define rightMotorF 9
#define rightMotorB 10
#define rightMotorPWM 11
#define leftMotorF 6
#define leftMotorB 7
#define leftMotorPWM 5
#define stby 8

QTRSensorsRC qtr((unsigned char[]) {12, A0, A1, A2, A3,4}, 6, 2500);
 
void setup()
{
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  Serial.begin(9600);
  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  } 
  
}

int lastError = 0;
float kp = 0.5;  // 0.08 // for small = 0.1
float kd = 0 ; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;
int Time, last_time=0;
int delta_time;
void loop()
{ unsigned int sensors[6];
  
  int position = qtr.readLine(sensors,QTR_EMITTERS_ON, 1);
   
    //Serial.println(position);
    int error = int(position) - 2500;        //for white line on black surface
    //Serial.println(error);
    integral += error;
    derivative = error - lastError;
    Time=millis();
    delta_time=Time-last_time;
    int power_difference = kp * error + ki * integral + kd * derivative/delta_time;
    //Serial.println(power_difference);
    lastError = error;
    last_time=Time;
    
    
    const int maximum =100; // Speed = 100 rpm
    
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;
      
    if (power_difference < 0) {
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, maximum);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, maximum + power_difference);
      digitalWrite(stby,HIGH);
    } 
    else { 
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, maximum - power_difference);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, maximum);
      digitalWrite(stby,HIGH);
    }
}
