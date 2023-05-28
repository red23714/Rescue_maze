#include "Robot.h"

Robot robot;

void setup()
{  
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  robot.init(false, false, false, false, true);
}

void loop()
{
  // robot.print_dis();
  // robot.print_gyro();

  robot.wait(1);
}
