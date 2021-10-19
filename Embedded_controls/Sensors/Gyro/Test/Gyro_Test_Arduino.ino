#include<Wire.h>

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
int16_t Tmp;

double x;
double y;
double z;
 


void setup(){
  
  //Power management
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  //Configure accelerometer
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  //Configure the acelerometer sensitivity
  Wire.write(0b00000000);
  Wire.endTransmission(true);
  
  
  Serial.begin(9600);
}
void loop(){
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  //Reading 14Bytes starting at 3B
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();


  x= RAD_TO_DEG * (atan2(AcY, AcZ));
  y= RAD_TO_DEG * (atan2(AcX, AcZ));
  z= RAD_TO_DEG * (atan2(AcY, AcX));
  

  
  Serial.print("AcX= ");
  Serial.println(AcX);
  Serial.print("AcY= ");
  Serial.println(AcY);
  Serial.print("AcZ= ");
  Serial.println(AcZ);
  
  //Serial.print("xAng= ");
  //Serial.println(xAng);

  

  //float Tmp_f = (Tmp/340.0f) + 36.53f;
  //Serial.print("Temp ");
  //Serial.println(Tmp_f);

  Serial.print("AngleX= ");
  Serial.println(x);
  Serial.print("AngleY= ");
  Serial.println(y);
  Serial.print("AngleZ= ");
  Serial.println(z);
  Serial.println("-----------------------------------------");
  delay(400);
}
