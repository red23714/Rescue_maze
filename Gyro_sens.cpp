#include "Gyro_sens.h"

void Gyro_sens::init_gyro()
{
    Wire.begin();
    delay(2000);

    if (!setup(sensor_gyro_newAdress)) // change to your own address
    {  
        while (1) 
        {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(5000);
        }
    }
    else Serial.println("MPU work");

    selectFilter(QuatFilterSel::MADGWICK_WITHOUT_MAG);
    //setFilterIterations(10);

    // calibrate anytime you want to
    verbose(true);
    calibrateAccelGyro();

    print_calibration();
    // verbose(false);
}

float Gyro_sens::adduction(float angle)
{
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;

    return angle;
}

float Gyro_sens::roll()
{
    float angle = getRoll() - roll_first;
    angle = adduction(angle);

    return angle;
}

float Gyro_sens::pitch()
{
    float angle = getPitch() - pitch_first;
    angle = adduction(angle);

    return angle;
}

float Gyro_sens::yaw()
{
    float angle = getYaw() - yaw_first;
    angle = adduction(angle);

    return angle;
}

void Gyro_sens::print_roll_pitch_yaw() 
{
    Serial.print("Angles: ");
    Serial.print(getYaw());
    Serial.print(" ");
    Serial.print(getPitch());
    Serial.print(" ");
    Serial.println(getRoll());
    delay(10);
}

void Gyro_sens::print_calibration() 
{
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(getMagBiasX());
    Serial.print(", ");
    Serial.print(getMagBiasY());
    Serial.print(", ");
    Serial.print(getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(getMagScaleX());
    Serial.print(", ");
    Serial.print(getMagScaleY());
    Serial.print(", ");
    Serial.print(getMagScaleZ());
    Serial.println();
}

void Gyro_sens::gyro_calibration(int port)
{
    if(port == -1) 
    {
        float timer_wait = millis();
        while(millis() - timer_wait < 5000)
        {
            update();
            Serial.println(yaw());
            delay(1);
        }
    }
    else
    {
        while(!digitalRead(port)) //!digitalRead(31)
        {
            update();
            Serial.println(yaw());
            delay(1);
        }
    }
    
    while (!update());

    roll_first = roll();
    pitch_first = pitch();
    yaw_first = yaw();
}

