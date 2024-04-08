#pragma once

#include "Updatable.h"
#include "Colors.h"
#include "Settings.h"
#include "Utils.h"
#include "Arduino.h"

class Color_sens : public Updatable
{
public:
    // Инициализация датчика цвета
    void init_color()
    {
        // сконфигурировать пины
        pinMode(color_S0, 1);
        pinMode(color_S1, 1);
        pinMode(color_S2, 1);
        pinMode(color_S3, 1);
        pinMode(color_OUT, 0);

        // масштабирование 20%
        digitalWrite(color_S0, 1);
        digitalWrite(color_S1, 0);

        Serial.println("Init color");
    }

    void update() override
    {
        color_in = read_color();
        if(color_in != old_color) counter++;
        if(counter > 10)
        {
            counter = 0;
            old_color = color_in;
        }
    }

    // Получение значения цвета на котором стоит робот
    color read_color()
    {
        // установить R фильтр
        digitalWrite(color_S2, 0);
        digitalWrite(color_S3, 0);

        // Получение частоты на выходе
        R = pulseIn(color_OUT, 0);

        // установить G фильтр
        digitalWrite(color_S2, 1);
        digitalWrite(color_S3, 1);

        // Получение частоты на выходе
        G = pulseIn(color_OUT, 0);

        // установить B фильтр
        digitalWrite(color_S2, 0);
        digitalWrite(color_S3, 1);

        // Получение частоты на выходе
        B = pulseIn(color_OUT, 0);

        // Serial.print(R);
        // Serial.print(" ");
        // Serial.print(G);
        // Serial.print(" ");
        // Serial.println(B);

        if (in_range(R, RED_BLUE, COLOR_SPREAD) && in_range(G, GREEN_BLUE, COLOR_SPREAD) &&
            in_range(B, BLUE_BLUE, COLOR_SPREAD))
        {
            return color::BLUE;
        }

        if (in_range(R, RED_BLACK, COLOR_SPREAD) && in_range(G, GREEN_BLACK, COLOR_SPREAD) &&
            in_range(B, BLUE_BLACK, COLOR_SPREAD))
        {
            return color::BLACK;
        }

        // if (in_range(R, RED_SILVER, COLOR_SPREAD) && in_range(G, GREEN_SILVER, COLOR_SPREAD) &&
        //     in_range(B, BLUE_SILVER, COLOR_SPREAD))
        // {
        //     return color::SILVER;
        // }

        return WHITE;
    }

    color get_color()
    {
        return old_color;
    }

    // Вывод показаний датчика цвета
    void print_color()
    {
        Serial.print("Color: ");
        switch (old_color)
        {
        case color::WHITE:
            Serial.print("White ");
            break;

        case color::BLUE:
            Serial.print("Blue ");
            break;

        case color::BLACK:
            Serial.print("Black ");
            break;

        case color::SILVER:
            Serial.print("Silver ");
            break;

        default:
            Serial.println("Color sens error");
            break;
        }
        Serial.print(R);
        Serial.print(" ");
        Serial.print(G);
        Serial.print(" ");
        Serial.println(B);
    }

private:
    color color_in = color::WHITE;
    color old_color = color::WHITE;

    int counter = 0;

    int R = 0;
    int G = 0;
    int B = 0;
};
