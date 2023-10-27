#pragma once

#include "Arduino.h"
#include <VL53L0X.h>
#include <Servo.h>

#include "Graph.h"
#include "Gyro_sens.h"
#include "Camera.h"
#include "Dalnometer.h"
#include "Color_sens.h"
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
    void print_color();
    void print_current_state();

    bool mov_forward();
    bool rotate(float);
    int rot_right();
    int rot_left();

    void motors(int, int);

    int giving(int, letter);

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
    Color_sens color_sens;

    Updatable * periph[6] = {&sensor_r, &sensor_u, &sensor_l, &mpu, &camera_l, &color_sens};

    state current_state = state::WAIT;
    int map_angle = 0;
    int map_angle_old = -1;
    int graph_length_old = -1;
    bool is_return_to = false;
    bool is_giving = false;
    bool is_stand = false;

    int right_dist = 0;
    int left_dist = 0;
    int central_dist = 0;

    int countL = 0;
    int countR = 0;

    void alg_right_hand();
    void alg_left_hand();
    void return_to_point();

    void init_servo();

    void init_encoder();
    static void encL();
    static void encR();
    void handleEncL();
    void handleEncR();

    void motor_l(int);
    void motor_r(int);
    void motor_stop();
};
