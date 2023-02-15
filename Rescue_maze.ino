#include <Wire.h>
#include <VL53L0X.h>
#include <Graph.h>
#include <MPU9250.h>
#include "state_machine.h"

#define DEBUG 0
#define DEBUG_ENC 0
#define DEBUG_HAND 0

#define M1_1 5
#define M1_2 6
#define M2_1 7
#define M2_2 8

#define SPEED 100

#define sensor_r_newAddress 41
#define sensor_u_newAddress 42
#define sensor_l_newAddress 43

#define sensor_gyro_newAdress 0x68

#define XSHUT_pin1 24
#define XSHUT_pin2 25
#define XSHUT_pin3 23

#define DISTANCE_WALL 110
#define DISTANCE 140
#define K_DIS 2
#define ROT_K 6
#define K_STOP_ROTATE 3

VL53L0X sensor_r;
VL53L0X sensor_u;
VL53L0X sensor_l;

MPU9250 mpu;

Graph graph;
byte x = 0, y = 0;

byte rotate_count = 0;
int angle = 0, angle_err = 0, roll_first = 0, pitch_first = 0, yaw_first = 0;

state current_state = WAIT;

int countL = 0, countR = 0;

void setup()
{
  pinMode(31, OUTPUT);
  Serial.begin(9600);
  init_dis();
  init_encoder();
  init_gyro();

  gyro_calibration();
}

void loop()
{
  if (mpu.update())
  {
    // right_hand();
    // mov_forward();
    state_machine();
    //motors(SPEED, SPEED);
    //debug_dis();
  }
}
