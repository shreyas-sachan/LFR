/* Program to test motors */

#define rightMotorF 7
#define rightMotorB 6
#define rightMotorPWM 5
#define leftMotorF 10
#define leftMotorB 11
#define leftMotorPWM 11
#define stby 8

void setup()
{
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
}
void loop() {
  digitalWrite(rightMotorF,HIGH);
  digitalWrite(rightMotorB,LOW);
  analogWrite(rightMotorPWM,75);
    digitalWrite(leftMotorF,HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 75);
  digitalWrite(stby,HIGH);

}

