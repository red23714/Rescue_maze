#pragma once

#include "Arduino.h"

#include "Gyro_sens.h"
#include "Camera.h"
#include "Dalnometer.h"
#include "Color_sens.h"
#include "Serva.h"
#include "Updatable.h"
#include "Static.h"

#include "Graph.h"
#include "States.h"
#include "Colors.h"
#include "Letters.h"
#include "Utils.h"
#include "Settings.h"

#define DEBUG 1

class Robot
{
public:
    void init();

    void state_machine();

    void wait(int);

    bool mov_forward();
    bool rotate(float);
    int rot_right();
    int rot_left();

    void print_dis();
    void print_enc();
    void print_map();
    void print_save();
    void print_gyro();
    void print_color();
    void print_current_state();

    void motors(int, int);

    void reset_robot();
private:
    Graph graph;

    static Robot *instance_;

    Dalnometer sensor_r = Dalnometer(XSHUT_pin_r, sensor_r_newAddress);
    Dalnometer sensor_u = Dalnometer(XSHUT_pin_u, sensor_u_newAddress);
    Dalnometer sensor_l = Dalnometer(XSHUT_pin_l, sensor_l_newAddress);

    Gyro_sens mpu = Gyro_sens(&Serial2);
    Camera camera_l = Camera(&Serial1, 1);
    Camera camera_r = Camera(&Serial3, 2);

    Serva myserva;
    Color_sens color_sens;

    Timers timers;
    Flags flags;

    state current_state = state::WAIT;
    state old_state = state::WAIT;
    Vec<state> path;
    int map_angle = 0;
    bool is_return_to = false;
    bool is_giving = false;
    bool is_stand = false;
    int side = 0;
    int side_giving = 0;

    int right_dist = 0;
    int left_dist = 0;
    int central_dist = 0;

    int countL = 0;
    int countR = 0;

    void alg_right_hand();
    void alg_left_hand();
    void return_to_point();

    void init_encoder();
    static void encL();
    static void encR();
    void handleEncL();
    void handleEncR();

    void motor_r(int);
    void motor_l(int);
    void motor_stop();
};
