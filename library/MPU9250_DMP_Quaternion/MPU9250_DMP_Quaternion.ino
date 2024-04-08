/************************************************************
  MPU9250_DMP_Quaternion
  Quaternion example for MPU-9250 DMP Arduino Library
  Jim Lindblom @ SparkFun Electronics
  original creation date: November 23, 2016
  https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

  The MPU-9250's digital motion processor (DMP) can calculate
  four unit quaternions, which can be used to represent the
  rotation of an object.

  This exmaple demonstrates how to configure the DMP to
  calculate quaternions, and prints them out to the serial
  monitor. It also calculates pitch, roll, and yaw from those
  values.

  Development environment specifics:
  Arduino IDE 1.6.12
  SparkFun 9DoF Razor IMU M0

  Supported Platforms:
  - ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB

MPU9250_DMP imu;


int adduction(int a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;

  return a;
}

void setup()
{
  SerialPort.begin(115200);
  Serial1.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | // Send accelerometer data
               DMP_FEATURE_GYRO_CAL       | // Calibrate the gyro data
               DMP_FEATURE_SEND_CAL_GYRO  | // Send calibrated gyro data
               DMP_FEATURE_6X_LP_QUAT     , // Calculate quat's with accel/gyro
               10);
  //imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
  //             DMP_FEATURE_GYRO_CAL, // Use gyro calibration
  //            10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

int mapYaw = 0;
int mapRoll = 0;

void loop()
{
  // Check for new data in the FIFO
  while (imu.fifoAvailable())
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      mapYaw = map(imu.yaw, 0, 360, 0, 254);
      float q0 = imu.calcQuat(imu.qw);
      float q1 = imu.calcQuat(imu.qx);
      float q2 = imu.calcQuat(imu.qy);
      float q3 = imu.calcQuat(imu.qz);
      float sinp = sqrt(1 + 2 * (q0 * q2 - q1 * q3));
      float cosp = sqrt(1 - 2 * (q0 * q2 - q1 * q3));
      float pitch = degrees(2 * atan2(sinp, cosp) - PI / 2);

      mapRoll = map(imu.pitch, 0, 360, 0, 254);
      //SerialPort.print((mapYaw << 8) + mapPitch);
      //SerialPort.print(" ");

      //Serial1.write(map(imu.roll, 0, 360, 0, 254));
      //Serial1.write()
      //Serial1.write(imu.yaw / 2);
      //Serial1.write(imu.pitch / 2);
      //Serial1.write(imu.yaw);
      //printIMUData();
    }
  }

  // SerialPort.print(mapYaw);
  // SerialPort.print(" ");
  // SerialPort.print(mapRoll);
  // SerialPort.print(" ");
  // SerialPort.println(crc8(mapYaw, mapRoll));


  Serial1.write(255);
  Serial1.write(mapYaw);
  Serial1.write(mapRoll);
  Serial1.write(crc8(mapYaw, mapRoll));
  delay(10);
}

byte crc8(int y, int p) {
  int data[2] = {y, p};
  int crc = 0xFF;
  for (byte i = 0; i < 2; ++i) {
    crc = crc ^ data[i];
    for (byte j = 0; j < 8; ++j) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else crc = crc << 1;
    }
  }

  return crc;
}

/*int crc8(int y, int p) {
  int data[2] = {y, p};
  int crc = 0xFF;

  for (int data_i = 0; data_i < 2; data_i++) {
    crc ^= data[data_i];
    for (int i = 15; i >= 8; i--) {
      if (getBit(crc, i)) {
        for (int j = 7; j >= 0; j--) {
          //cout << (bit(gen, j) << (i + j - 7)) << endl;
          crc ^= (getBit(gen, j) << (i + j - 7));
        }
      }
    }
  }
  }*/

void printIMUData(void)
{
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

  SerialPort.println("Q: " + String(q0, 4) + ", " +
                     String(q1, 4) + ", " + String(q2, 4) +
                     ", " + String(q3, 4));
  SerialPort.println("R/P/Y (NORM): " + String(adduction(imu.roll)) + ", "
                     + String(adduction(imu.pitch)) + ", " + String(adduction(imu.yaw)));
  //SerialPort.println(sign() * degrees(acos(cos(radians(imu.pitch)) * cos(radians(imu.roll)))));
  //degrees(atan2(2 * q1 * q0 - 2 * q2 * q3, 1 - 2 * q1 * q1 - 2 * q3 * q3));
  //degrees(acos(-cos(radians(imu.yaw)) * sin(radians(imu.roll)) + sin(radians(imu.yaw)) * sin(radians(imu.pitch)) * cos(radians(imu.roll))));
  float sinp = sqrt(1 + 2 * (q0 * q2 - q1 * q3));
  float cosp = sqrt(1 - 2 * (q0 * q2 - q1 * q3));
  float pitch = degrees(2 * atan2(sinp, cosp) - PI / 2);

  SerialPort.println(pitch);
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}

bool getBit(int a, byte n) {
  return a & (1 << n);
}

int sign(int a) {
  if (a < 90) return 1;
  else return -1;
}
