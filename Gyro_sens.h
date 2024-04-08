#pragma once

#include "Letters.h"
#include "Updatable.h"
#include "Utils.h"
#include "Arduino.h"

class Gyro_sens : public Updatable
{
public:
    Gyro_sens(HardwareSerial *Serial_port) : s(Serial_port)
    {
        s->begin(115200);
    }

    void init_gyro()
    {
        Serial.println("Init gyro");

        float timer_wait = millis();
        while (millis() - timer_wait < 15000)
        {
            update();
            Serial.println(yaw);
            delay(1);
        }

        reset_yaw();
        pitch_first = pitch;
    }

    void update() override
    {
        long long timer = millis();

        while(s->available() >= 4 && millis() - timer < 100)
        {
            if(s->read() == 255)
            {
                unsigned char a[2] = {s->read(), s->read()};

                if(Crc8(a, 2) == s->read())
                {
                    yaw = map(a[0], 0, 254, -180, 180);
                    pitch = adduction(map(a[1], 0, 254, 0, 360));
                }
            }
        }
    }

    int inline get_yaw() { return adduction(yaw - yaw_first); }
    int inline get_pitch() { return pitch; }
    int inline get_pitch_first() { return pitch_first; }

    void reset_yaw()
    {
        update();
        yaw_first = adduction(get_yaw() + yaw_first);
        pitch_first = pitch;
    }

    void print_gyro()
    {
        Serial.print("Yaw: ");
        Serial.print(get_yaw());
        Serial.print(" Pitch: ");
        Serial.println(get_pitch());
    }

private:
    int yaw = 0;
    int pitch = 0;
    int yaw_first = 0;
    int pitch_first = 0;

    bool is_yaw = false;
    HardwareSerial *s;

    unsigned char Crc8(unsigned char *pcBlock, unsigned int len)
    {
        unsigned char crc = 0xFF;
        unsigned int i;

        while (len--)
        {
            crc ^= *pcBlock++;

            for (i = 0; i < 8; i++)
                crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
        }

        return crc;
    }
};
