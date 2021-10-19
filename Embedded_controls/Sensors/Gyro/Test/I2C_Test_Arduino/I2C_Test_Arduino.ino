#include "i2c.h"
#include "gyro.h"

void setup() {
  SetupGyroSensor();
}

void loop()
{
  Serial.begin(9600);


  struct GyroData data = GetGyroSensorData();

  Serial.print("AngleX= ");
  Serial.println(data.rotationX);
  Serial.print("AngleY= ");
  Serial.println(data.rotationY);
  Serial.print("AngleZ= ");
  Serial.println(data.rotationZ);
  Serial.println("-----------------------------------------");


  delay(400);
}
