#pragma once

#include "Letters.h"
#include "Updatable.h"
#include "Arduino.h"

class Gyro_sens : public Updatable
{
public:
    Gyro_sens(HardwareSerial *Serial) : s(Serial)
    {
        s->begin(115200);
    }

    void update() override
    {
        if(s->available())
        {
          int n = s->read();
          if(n == 255) is_yaw = true;
          if (is_yaw && n != 255) 
          {
            yaw = map(n, 0, 254, 0, 360);
            is_yaw = false;
          }
          if(!is_yaw && n != 255) pitch = map(n, 0, 254, -90, 90);
        }
    }

    int inline get_yaw() { return yaw - yaw_first; }
    int inline get_pitch() { return pitch - pitch_first; }

    void set_yaw_first(int value) { yaw_first = value; }
    void set_pitch_first(int value) { pitch_first = value; }

    void print_gyro()
    {
        Serial.print("Yaw: ");
        Serial.print(yaw);
        Serial.print(" Pitch: ");
        Serial.println(pitch);
    }

private:
    int yaw = 0;
    int pitch = 0;
    int yaw_first = 0;
    int pitch_first = 0;

    bool is_yaw = false;
    HardwareSerial *s;
};
