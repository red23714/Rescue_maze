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
        if (s->available() && is_update)
        {
            int n = s->read();
            switch (n)
            {
            case 48:
                letter_detect = N;
                break;
            case 72:
                letter_detect = H;
                break;
            case 83:
                letter_detect = S;
                break;
            case 85:
                letter_detect = U;
                break;
            case 71:
                letter_detect = GREEN;
                break;
            case 82:
                letter_detect = RED;
                break;
            case 89:
                letter_detect = YELLOW;
                break;
            default:
                letter_detect = 1;
                break;
            }
        }
    }
    letter inline get_letter() { return letter_detect; }
    void set_is_update(bool value) { is_update = value; }
    int inline get_side() { return side; }

    void print_camera()
    {
        Serial.print("Side: ");

        if (side == 1)
            Serial.print("Left ");
        else
            Serial.print("Right ");

        Serial.print("Letter: ");

        switch (letter_detect)
        {
        case letter::N:
            Serial.println("N");
            break;

        case letter::H:
            Serial.println("H");
            break;

        case letter::S:
            Serial.println("S");
            break;

        case letter::U:
            Serial.println("U");
            break;

        case letter::GREEN:
            Serial.println("Green");
            break;

        case letter::RED:
            Serial.println("Red");
            break;

        case letter::YELLOW:
            Serial.println("Yellow");
            break;

        default:
            Serial.println("Camera error");
            break;
        }
    }

private:
    letter letter_detect;
    HardwareSerial *s;
    int side = 0;
    bool is_update = true;
};
