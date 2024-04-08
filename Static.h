#include "Arduino.h"

struct Timers
{
    float timer_rotate = millis();
    float timer_i_rotate = millis();

    void reset()
    {
        timer_rotate = millis();
        timer_i_rotate = millis();
    }
};

struct Flags
{
    bool flag_rotate = true;

    void reset()
    {
        flag_rotate = true;
    }
};