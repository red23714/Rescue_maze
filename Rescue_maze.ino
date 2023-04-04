#include <Wire.h>
#include <VL53L0X.h>
#include <Graph.h>
#include <MPU9250.h>
#include "state_machine.h"

#define DEBUG 0
#define DEBUG_ENC 0
#define DEBUG_HAND 0

#define LONG_RANGE

#define M1_1 5
#define M1_2 6
#define M2_1 7
#define M2_2 8

#define SPEED 100

#define sensor_r_newAddress 42
#define sensor_u_newAddress 43
#define sensor_l_newAddress 44
#define sensor_b_newAddress 45

#define sensor_gyro_newAdress 0x68

#define XSHUT_pin1 23
#define XSHUT_pin2 24
#define XSHUT_pin3 25
#define XSHUT_pin4 26

#define DISTANCE_WALL 120 // 120
#define DISTANCE 200 //170
#define CELL_SIZE 350 // 350
#define K_WALL 1.5
#define K_WALL_I 5
#define K_CALIBRATION 20
#define K_DIS 1
#define ROT_K 10 // 6
#define K_STOP_ROTATE 3

VL53L0X sensor_r;
VL53L0X sensor_u;
VL53L0X sensor_l;
VL53L0X sensor_b;

MPU9250 mpu;

Graph graph;
int x = 0, y = 0;

// byte rotate_count = 0;
int angle = 0, angle_err = 0, roll_first = 0, pitch_first = 0, yaw_first = 0;

state current_state = WAIT;

int distance_old = 0;
int vlFlag1 = 0, vlFlag2 = 0, vlFlag3 = 0;

int countL = 0, countR = 0, count_old;

void setup()
{
  pinMode(31, OUTPUT);
  Serial.begin(115200);
  init_dis();
  init_encoder();
  init_gyro();

  gyro_calibration();

  if(get_distance(&sensor_u) > 1000) distance_old = get_distance(&sensor_u);
  else distance_old = get_distance(&sensor_b);
}

void loop()
{
  int map_angle = 0;

  while(true)
  {
    state_machine(&map_angle);
    // Serial.print(get_distance(&sensor_u));
    // Serial.print(" ");
    // Serial.print(distance_old);
    // Serial.print(" ");
    // Serial.println(get_distance(&sensor_b));
    // debug_dis();

    wait(1);
  }
}
