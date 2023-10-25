#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

#include "Updatable.h"

#define sensor_gyro_newAdress 0x68

class Gyro_sens : public MPU9250, public Updatable
{
public:
  Gyro_sens() {

  };
  int roll_first = 0, pitch_first = 0, yaw_first = 0;

  void init_gyro();

  float roll();
  float pitch();
  float yaw();

  void print_roll_pitch_yaw();
  void print_calibration();

  void gyro_calibration(int);

  void update() override;

  int get_yaw();
  int get_pitch();
  int get_roll();

private:
  float adduction(float);

  int roll_angle = 0;
  int yaw_angle = 0;
  int pitch_angle = 0;
};
