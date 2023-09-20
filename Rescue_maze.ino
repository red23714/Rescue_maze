#include "Robot.h"

Robot robot;

void setup()
{  
  Serial.begin(115200);
  Serial1.begin(9600);
  // Serial2.begin(9600);

  robot.init(false, false, false, true, false, false);
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
  
  robot.wait(1);

  robot.state_machine();
}
