#include "Robot.h"

Robot robot;

void setup()
{  
  Serial.begin(115200);
  // Serial1.begin(9600);
  // Serial2.begin(9600);

  robot.init(true, true, true, true, true, true);
}

void loop()
{
  robot.print_dis();
  // robot.print_gyro();
  // robot.print_enc();
  
  robot.wait(1);

  robot.state_machine();
}
