#include "Robot.h"

Robot robot;

void setup()
{  
  Serial.begin(115200);

  robot.init(true, true, true, true, true, true);
}

void loop()
{
  robot.print_dis();
  robot.print_gyro();
  robot.print_save();
  // robot.print_enc();

  // if(Serial2.available())
  // {
  //   Serial.println(Serial2.read());
  // }
  
  // robot.wait(1);

  // robot.state_machine();
}
