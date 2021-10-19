#ifndef gyro_h
#define gyro_h
#include <stdio.h>
#include <math.h>

#define RAD_TO_DEG 57.2957795130823
#define MPU_addr 0x68

struct GyroData
{
  int16_t accelerationX;
  int16_t accelerationY;
  int16_t accelerationZ;
  int16_t temprature;
  int16_t rotAccelX;
  int16_t rotAccelY;
  int16_t rotAccelZ;
  double rotationX;
  double rotationY;
  double rotationZ;
};
void SetupGyroSensor();
struct GyroData GetGyroSensorData();

#endif