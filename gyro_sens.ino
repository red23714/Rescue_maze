void init_gyro()
{
    Wire.begin();
    delay(2000);

    if (!mpu.setup(sensor_gyro_newAdress)) // change to your own address
    {  
      while (1) 
      {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(5000);
      }
    }

    mpu.selectFilter(QuatFilterSel::MADGWICK_WITHOUT_MAG);
    //mpu.setFilterIterations(10);

    // calibrate anytime you want to
    mpu.verbose(true);
    mpu.calibrateAccelGyro();

    print_calibration();
    mpu.verbose(false);
}

float roll()
{
  float angle = mpu.getRoll() - roll_first;
  angle = adduction(angle);

  return angle;
}

float pitch()
{
  float angle = mpu.getPitch() - pitch_first;
  angle = adduction(angle);

  return angle;
}

float yaw()
{
  float angle = mpu.getYaw() - yaw_first;
  angle = adduction(angle);

  return angle;
}

void print_roll_pitch_yaw() 
{
  Serial.print("Acc: ");
  Serial.print(mpu.getAccX());
  Serial.print(" ");
  Serial.print(mpu.getAccY());
  Serial.print(" ");
  Serial.print(mpu.getAccZ());
  Serial.print(" ");
  Serial.print("Gyro: ");
  Serial.print(mpu.getGyroX());
  Serial.print(" ");
  Serial.print(mpu.getGyroY());
  Serial.print(" ");
  Serial.print(mpu.getGyroZ());
  Serial.print(" ");
  Serial.print("Angles: ");
  Serial.print(mpu.getYaw());
  Serial.print(" ");
  Serial.print(mpu.getPitch());
  Serial.print(" ");
  Serial.println(mpu.getRoll());
  delay(10);
}

void print_calibration() 
{
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void gyro_calibration()
{
  while(!digitalRead(31))
  {
    mpu.update();
    Serial.println(yaw());
    delay(1);
  }
  
  while (!mpu.update());

  roll_first = roll();
  pitch_first = pitch();
  yaw_first = yaw();
  angle_err = 0;
}
