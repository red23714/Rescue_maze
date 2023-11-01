#pragma once
#include <Servo.h>
#include "Settings.h"

class Serva : public Servo
{
public:
    // Инициализация сервы
    void init_servo()
    {
        attach(SERVO_PIN);
        start_pos();
        Serial.println("Init servo");
    }

    void start_pos()
    {
        write(START_POS_SERVO);
    }

    void sweep(int start_pos, int end_pos, int step)
    {
        if (end_pos - start_pos > 0)
        {
            for (int i = start_pos; i < end_pos; i++)
            {
                write(i);
                delay(step);
            }
        }
        else
        {
            for (int i = start_pos; i > end_pos; i--)
            {
                write(i);
                delay(step);
            }
        }
        delay(200);
    }

    // Выдача спаснабора
    int giving(int side_in, letter count_save)
    {
        long long timer = millis();

        int l = 0;
        switch (count_save)
        {
        case letter::H:
            l = 3;
            break;

        case letter::S:
            l = 2;
            break;

        case letter::RED:
            l = 1;
            break;

        case letter::YELLOW:
            l = 1;
            break;

        default:
            break;
        }

        for (int n = 0; n < l; n++)
        {
            digitalWrite(LED_B, HIGH);

            if (side_in == 1)
            {
                sweep(START_POS_SERVO, 160, 5);
                digitalWrite(LED_B, LOW);
                sweep(160, START_POS_SERVO, 5);
            }
            else if (side_in == 2)
            {
                sweep(START_POS_SERVO, 0, 5);
                digitalWrite(LED_B, LOW);
                sweep(0, START_POS_SERVO, 5);
            }
        }

        if (l == 0) digitalWrite(LED_B, HIGH);
        while (millis() - timer < 5000);
        if (l == 0) digitalWrite(LED_B, LOW);
    }
};
