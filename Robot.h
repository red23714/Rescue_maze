#pragma once

#include <Arduino.h>
#include <VL53L0X.h>
#include <Servo.h>

#include "Graph.h"
#include "Gyro_sens.h"

#include "States.h"
#include "Colors.h"
#include "Utils.h"
#include "Settings.h"

#define DEBUG 0

class Robot
{  
public:
    void init(bool is_button = true, bool is_mpu = true, bool is_dis = true, 
              bool is_enc = true, bool is_servo = true, bool is_color = true);

    void state_machine();

    void print_dis();
    void print_enc();
    void print_map();
    void print_save();
    void print_gyro();

    bool mov_forward();
    bool rotate(float);
    int rot_right();
    int rot_left();

    void motors(int, int);

    void wait(int);
private:
    Graph graph;

    static Robot * instance_;

    VL53L0X sensor_r;
    VL53L0X sensor_u;
    VL53L0X sensor_l;

    Gyro_sens mpu;

    Servo myservo;

    state current_state = WAIT;
    int map_angle = 0;
    bool is_return_to = false;

    int right_dist = 0;
    int left_dist = 0;
    int central_dist = 0;

    color current_color = WHITE;

    int countL = 0;
    int countR = 0;

    int side = 0;
    int count_save = 0;

    void alg_right_hand();
    void alg_left_hand();
    void return_to_point();

    void init_servo();
    int giving(); 

    void init_dis();
    int get_distance(VL53L0X); 

    void init_color();
    color get_color();

    void init_encoder();
    static void encL();
    static void encR();
    void handleEncL();
    void handleEncR();
    void getCountL();
    void getCountR();

    void motor_l(int);
    void motor_r(int);
    void motor_stop();
};
