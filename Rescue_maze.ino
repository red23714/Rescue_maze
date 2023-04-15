#include "Robot.h"

Robot robot;

void setup()
{  
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  robot.init();
}

void loop()
{
    robot.state_machine();
}
