#pragma once

#include "Letters.h"
#include "Colors.h"
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
            switch (n)
            {
            case 48:
                letter = N;
                break;
            case 72:
                letter = H;
                break;
            case 83:
                letter = S;
                break;
            case 85:
                letter = U;
                break;
            case 71:
                color = GREEN;
                break;
            case 82:
                color = RED;
                break;
            case 89:
                color = YELLOW;
                break;
            default:
                letter = N;
                break;
            }
        }
    }
    letter inline get_letter() { return letter; }

    int inline get_side() { return side; }

private:
    letter letter;
    color color;
    HardwareSerial *s;
    int side = 0;
};
