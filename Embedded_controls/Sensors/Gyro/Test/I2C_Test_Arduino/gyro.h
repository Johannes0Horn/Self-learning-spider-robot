#ifndef gyro_h
#define gyro_h

const int MPU_addr = 0x68;

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



void SetupGyroSensor() {
  i2c_init();
  int error = 0;
  //Power management
  {
    error = i2c_start(MPU_addr << 1);
    error = i2c_write(0x6B);
    error = i2c_write(0);
  }

  //Set sensitivity of the accelerometer
  {
    error = i2c_start(MPU_addr << 1);
    error = i2c_write(0x1C);
    error = i2c_write(0b00001000);
  }
}
struct GyroData GetGyroSensorData()
{
  {
    int error = i2c_start(MPU_addr << 1);
    error = i2c_write(0x3B);
    i2c_stop();
  }

  struct GyroData gyroData;
  
  {
    uint8_t data[14];
    int error = i2c_receive(MPU_addr << 1, data, 14);
    gyroData.accelerationX = (data[0] << 8) | data[1];
    gyroData.accelerationY = (data[2] << 8) | data[3];
    gyroData.accelerationZ = (data[4] << 8) | data[5];
    gyroData.temprature = (data[6] << 8) | data[7];
    gyroData.rotAccelX = (data[8] << 8) | data[9];
    gyroData.rotAccelY = (data[10] << 8) | data[11];
    gyroData.rotAccelZ = (data[12] << 8) | data[13];
  }


  {
    gyroData.rotationX = RAD_TO_DEG * (atan2(gyroData.accelerationY, gyroData.accelerationZ));
    gyroData.rotationY = RAD_TO_DEG * (atan2(gyroData.accelerationX, gyroData.accelerationZ));
    gyroData.rotationZ = RAD_TO_DEG * (atan2(gyroData.accelerationY, gyroData.accelerationX));
  }
  return gyroData;
}


#endif