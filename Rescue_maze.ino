#include "Robot.h"

Robot robot;

void setup()
{  
  Serial.begin(115200);

  robot.init();
}

void loop()
{
  // robot.print_dis();
  // robot.print_gyro();
  // robot.print_save();
  // robot.print_enc();
  // robot.print_color();
  // robot.print_current_state();
  // robot.print_map();

  // robot.wait(1);

  // robot.state_machine();
}
