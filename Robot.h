#pragma once

#include <Arduino.h>
#include <VL53L0X.h>
#include <Servo.h>

#include "Graph.h"
#include "Gyro_sens.h"
#include "data.h"
#include "state_machine.h"
#include "utils.h"

#define LONG_RANGE

#define M1_1 5
#define M1_2 6
#define M2_1 7
#define M2_2 8

#define sensor_r_newAddress 42
#define sensor_u_newAddress 43
#define sensor_l_newAddress 44

#define BUTTON_PIN 31

#define XSHUT_pin1 23
#define XSHUT_pin2 24
#define XSHUT_pin3 25

#define SPEED 100

#define DISTANCE_WALL 120 // 120
#define DISTANCE 170 //170

// #define CELL_SIZE 250 // 350
#define CELL_SIZE_ENCODER 1700

#define K_WALL 1.5 // 1.5
#define K_WALL_I 5 // 5
#define K_CALIBRATION 20 // 20
#define K_ROT 10 // 10
#define K_STOP_ROTATE 3 // 3

class Robot
{  
public:
    int init();
    void state_machine();
private:
    Gyro_sens mpu;
    Graph graph;
    static Robot * instance_;
    state current_state = WAIT;

    int map_angle = 0;
    bool is_return_to = false;

    int right_dist = 0;
    int left_dist = 0;
    int central_dist = 0;

    VL53L0X sensor_r;
    VL53L0X sensor_u;
    VL53L0X sensor_l;

    Servo myservo;

    int countL = 0;
    int countR = 0;

    bool mov_forward();
    bool rotate(float);
    int rot_right();
    int rot_left();

    void alg_right_hand();
    void alg_left_hand();
    void return_to_point();

    void init_servo();
    int giving(); 

    void wait(int);

    void init_dis();
    int get_distance(VL53L0X); 
    void debug_dis();

    void init_encoder();
    static void encL();
    static void encR();
    void handleEncL();
    void handleEncR();

    void motor_l(int);
    void motor_r(int);
    void motors(int, int);
    void motor_stop();
};
