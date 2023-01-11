#include <Wire.h>
#include <VL53L0X.h>
#include <Graph.h>

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

#define XSHUT_pin1 24
#define XSHUT_pin2 25
#define XSHUT_pin3 23

#define DISTANCE 120

VL53L0X sensor_r;
VL53L0X sensor_u;
VL53L0X sensor_l;

Graph graph;
byte x = 0, y = 0;

byte rotate_count = 0;
int angle;

int countL = 0, countR = 0;

void setup() 
{
  Serial.begin(9600);
  init_dis();
  init_encoder();
}

int test = 0;

void loop() 
{
  right_hand();
  //debug_dis();
}

