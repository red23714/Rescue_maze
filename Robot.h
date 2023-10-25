#pragma once

#include "Arduino.h"
#include <VL53L0X.h>
#include <Servo.h>

#include "Graph.h"
#include "Gyro_sens.h"
#include "Camera.h"
#include "Dalnometer.h"
#include "Updatable.h"

#include "States.h"
#include "Colors.h"
#include "Letters.h"
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

    int giving(int, int);

    void wait(int);

private:
    Graph graph;

    static Robot *instance_;

    Dalnometer sensor_r = Dalnometer(XSHUT_pin_r, sensor_r_newAddress);
    Dalnometer sensor_u = Dalnometer(XSHUT_pin_u, sensor_u_newAddress);
    Dalnometer sensor_l = Dalnometer(XSHUT_pin_l, sensor_l_newAddress);

    Gyro_sens mpu;

    Camera camera_l = Camera(&Serial1, 1);

    Servo myservo;

    Updatable * periph[5] = {&sensor_r, &sensor_u, &sensor_l, &mpu, &camera_l};

    state current_state = WAIT;
    state old_state = WAIT;
    int map_angle = 0;
    int map_angle_old = 0;
    int graph_length_old = 0;
    bool is_return_to = false;

    bool is_giving = false;

    int right_dist = 0;
    int left_dist = 0;
    int central_dist = 0;

    color current_color = WHITE;

    int countL = 0;
    int countR = 0;

    void alg_right_hand();
    void alg_left_hand();
    void return_to_point();

    void init_servo();

    void init_color();
    color get_color();

    void init_encoder();
    static void encL();
    static void encR();
    void handleEncL();
    void handleEncR();

    void motor_l(int);
    void motor_r(int);
    void motor_stop();
};
