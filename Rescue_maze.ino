#include "Robot.h"

Robot robot;

void setup()
{  
  Serial.begin(115200);
  Serial1.begin(9600);
  // Serial2.begin(9600);

  robot.init(false, true, true, true, false, false);
}

void loop()
{
  // robot.print_dis();
  // robot.print_gyro();
  // robot.print_enc();

  // if(Serial2.available())
  // {
  //   Serial.println(Serial2.read());
  // }

  if(robot.mov_forward()) delay(1000);
  
  robot.wait(1);

  // robot.state_machine();
}
