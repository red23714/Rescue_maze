#pragma once

#include "Letters.h"
#include "Updatable.h"
#include "Arduino.h"

class Camera : public Updatable
{
public:
    Camera(HardwareSerial *Serial, int side) : s(Serial), side(side)
    {
        s->begin(115200);
    }

    void update() override
    {
        if (s->available())
        {
            int n = s->read();
            if(is_update) letter_detect = (letter) n;
        }
    }
    
    letter inline get_letter() { return letter_detect; }
    void set_is_update(bool flag) { is_update = flag; letter_detect = letter::N; }
    int inline get_side() { return side; }

    void print_camera()
    {
        Serial.print("Side: ");

        if (side == 1) Serial.print("Left ");
        else Serial.print("Right ");

        Serial.print("Letter: ");

        switch (letter_detect)
        {
        case letter::N:
            Serial.print("N");
            break;

        case letter::H:
            Serial.print("H");
            break;

        case letter::S:
            Serial.print("S");
            break;

        case letter::U:
            Serial.print("U");
            break;

        case letter::GREEN:
            Serial.print("Green");
            break;

        case letter::RED:
            Serial.print("Red");
            break;

        case letter::YELLOW:
            Serial.print("Yellow");
            break;

        default:
            Serial.print("Camera error");
            break;
        }
    }

private:
    letter letter_detect = letter::N;
    bool is_update = true;
    HardwareSerial *s;
    int side = 0;
};
