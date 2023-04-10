#include <Wire.h>
#include <VL53L0X.h>
#include <Graph.h>
#include <Gyro_sens.h>
#include <Servo.h>
#include "state_machine.h"
#include "data.h"

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

#define XSHUT_pin1 23
#define XSHUT_pin2 24
#define XSHUT_pin3 25

#define DISTANCE_WALL 120 // 120
#define DISTANCE 170 //170

// #define CELL_SIZE 250 // 350
#define CELL_SIZE_ENCODER 1700

#define K_WALL 1.5 // 1.5
#define K_WALL_I 5 // 5
#define K_CALIBRATION 20 // 20
#define K_ROT 10 // 10
#define K_STOP_ROTATE 3 // 3

VL53L0X sensor_r;
VL53L0X sensor_u;
VL53L0X sensor_l;

Gyro_sens mpu;

Servo myservo;

Graph graph;

int countL = 0, countR = 0;

void add_by_angle(Map_data*, bool = true);

void setup()
{
  pinMode(31, OUTPUT);
  
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  init_dis();
  init_encoder();
  mpu.init_gyro();

  myservo.attach(9);
  myservo.write(91);

  mpu.gyro_calibration();

  // graph.add_node(0, 1);
  // graph.add_node(1, 1);
  // graph.add_node(1, 2);
  // graph.add_node(2, 1, false);
  // graph.add_node(3, 1);
  // graph.add_node(3, 2);
  // graph.add_node(3, 0);
  // graph.add_node(2, 0);
  // graph.add_node(1, 0);
  // graph.add_node(4, 1);
}

void loop()
{
  state current_state = WAIT;

  Map_data map_data;
  Package_data package_data;
  Distance_data distance_data;

  while(true)
  {
    state_machine(&map_data, &package_data, &distance_data, &current_state);
    
    // if(x == 5 && y == 1) return_to_point(&map_angle);

    // debug_dis();

    wait(1, &distance_data);
  }
}
