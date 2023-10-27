#include "Robot.h"

#define IS_INIT_BUTTON true
#define IS_INIT_MPU9250 true
#define IS_INIT_VL53L0X true
#define IS_INIT_ENCODERS true
#define IS_INIT_SERVO true
#define IS_INIT_COLOR true

Robot robot;

void setup()
{  
  Serial.begin(115200);

  robot.init(IS_INIT_BUTTON, IS_INIT_VL53L0X, IS_INIT_ENCODERS, IS_INIT_SERVO, IS_INIT_COLOR);

}

void loop()
{
  // robot.print_dis();
  // robot.print_gyro();
  // robot.print_save();
  // robot.print_enc();
  // robot.print_color();
  robot.print_current_state();

  robot.wait(1);

  robot.state_machine();
}
