#include <AccelStepper.h>

const int ena_Z = 11;
const int dir_Z = 10;
const int pul_Z = 9;
const int limit_Z = 8;

const int ena_Y = 6;
const int dir_Y = 5;
const int pul_Y = 3;
const int limit_Y = 2;

AccelStepper zStepTest(AccelStepper::DRIVER, pul_Z, dir_Z);
AccelStepper yStepTest(AccelStepper::DRIVER, pul_Y, dir_Y);

void setup() 
{
  pinMode(pul_Z, OUTPUT);
  pinMode(dir_Z, OUTPUT);
  pinMode(ena_Z, OUTPUT);
  digitalWrite(ena_Z, LOW);
  //zStepTest.setMaxSpeed(60000);
  zStepTest.setMaxSpeed(1000);
  zStepTest.setAcceleration(1000);
  //zStepTest.moveTo(100);

  pinMode(pul_Y, OUTPUT);
  pinMode(dir_Y, OUTPUT);
  pinMode(ena_Y, OUTPUT);
  digitalWrite(ena_Y, LOW);
  //yStepTest.setMaxSpeed(80000);
  yStepTest.setMaxSpeed(8000);
  yStepTest.setAcceleration(20000);
  yStepTest.moveTo(100);
  Serial.begin(115200);
}

void loop() 
{
  //yStepTest.run();
  for (int i = 0; i<1000; i++)
  {
    zStepTest.moveTo(i);
    zStepTest.run();
    Serial.println(i);
  }
  delay(1000);
  

 /* if(!zStepTest.run()){
    zStepTest.moveTo(-zStepTest.currentPosition());
  }
  
  if(!yStepTest.run()){
    yStepTest.moveTo(-yStepTest.currentPosition());
  }
*/
}
