#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

#define sensor_gyro_newAdress 0x68

class Gyro_sens : public MPU9250
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

  void gyro_calibration();

private:
  float adduction(float);
};
