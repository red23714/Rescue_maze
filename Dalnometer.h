#pragma once

#include <VL53L0X.h>
#include <Wire.h>
#include "Updatable.h"

class Dalnometer : public VL53L0X, public Updatable
{
public:
    Dalnometer(int XSHUT_pin, int sensor_newAddress, bool LONG_RANGE = false) 
    : 
        XSHUT_pin(XSHUT_pin), 
        sensor_newAddress(sensor_newAddress),
        LONG_RANGE(LONG_RANGE)
    {}

    // Инициализация датчиков расстояния
    void init_dis()
    {
        pinMode(XSHUT_pin, OUTPUT);

        digitalWrite(XSHUT_pin, 0);

        Wire.begin();
        delay(500);

        digitalWrite(XSHUT_pin, 1);
        delay(100);
        VL53L0X::setAddress(sensor_newAddress);
        delay(10);

        VL53L0X::init();

        delay(2000);

        VL53L0X::setTimeout(500);

        VL53L0X::startContinuous();

        if (LONG_RANGE)
        {
            // lower the return signal rate limit (default is 0.25 MCPS)
            VL53L0X::setSignalRateLimit(0.1);
        }
    }

    // Функция для получения расстояния с датчиков
    int get_distance()
    {
        if (VL53L0X::timeoutOccurred()) Serial.print(" TIMEOUT");
        sensor_dis = VL53L0X::readRangeContinuousMillimeters(); //-50

        if (sensor_dis == 8191)
            return -1;

        if (sensor_dis == 8190)
            sensor_dis = sensor_dis_old;
        else
            sensor_dis_old = sensor_dis;

        return sensor_dis;
    }

    void update() override
    {
        sensor_dis = get_distance();
    }

    int get_sensor_dis()
    {
        return sensor_dis;
    }

private:
    int sensor_dis = 0;
    int sensor_dis_old = 0;

    int XSHUT_pin = 0;
    int sensor_newAddress = 0;
    bool LONG_RANGE = false;
};
