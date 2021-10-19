#include <avr/io.h>
#include <util/twi.h>
#include <stdint.h>


//i2c from https://github.com/knightshrub/I2C-master-lib/blob/master/i2c_master.c
#ifndef  F_CPU
#define F_CPU 16000000UL
#endif
#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)
#define I2C_READ 0x01
#define I2C_WRITE 0x00

void i2c_init(void)
{
  TWBR = (uint8_t)TWBR_val;
}

uint8_t i2c_start(uint8_t address)
{
  // reset TWI control register
  TWCR = 0;
  // transmit START condition 
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );
  
  // check if the start condition was successfully transmitted
  if((TWSR & 0xF8) != TW_START){ return 1; }
  
  // load slave address into data register
  TWDR = address;
  // start transmission of address
  TWCR = (1<<TWINT) | (1<<TWEN);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );
  
  // check if the device has acknowledged the READ / WRITE mode
  uint8_t twst = TW_STATUS & 0xF8;
  if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
  
  return 0;
}

uint8_t i2c_write(uint8_t data)
{
  // load data into data register
  TWDR = data;
  // start transmission of data
  TWCR = (1<<TWINT) | (1<<TWEN);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );
  
  if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
  
  return 0;
}

void i2c_stop(void)
{
  // transmit STOP condition
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}

uint8_t i2c_read_ack(void)
{
  
  // start TWI module and acknowledge data after reception
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); 
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );
  // return received data from TWDR
  return TWDR;
}

uint8_t i2c_read_nack(void)
{
  
  // start receiving without acknowledging reception
  TWCR = (1<<TWINT) | (1<<TWEN);
  // wait for end of transmission
  while( !(TWCR & (1<<TWINT)) );
  // return received data from TWDR
  return TWDR;
}

uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length)
{
  if (i2c_start(address | I2C_READ)) return 1;
  
  for (uint16_t i = 0; i < (length-1); i++)
  {
    data[i] = i2c_read_ack();
  }
  data[(length-1)] = i2c_read_nack();
  
  i2c_stop();
  
  return 0;
}


const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
int16_t Tmp;
double x;
double y;
double z;


void setup() {

  Serial.begin(9600);
  Serial.println("Setup started");
  int error=0;

  i2c_init();

  //Power management
  {
    error = i2c_start(MPU_addr<<1);
    if(error)
    {
       Serial.println("Error1");
    }
  
    error = i2c_write(0x6B);
    if(error)
    {
       Serial.println("Error2");
    }
  
    error = i2c_write(0);
    if(error)
    {
       Serial.println("Error3");
    }
    i2c_stop();
  }

  //Set sensitivity of the accelerometer
  { 
    error = i2c_start(MPU_addr<<1);
    if(error)
    {
       Serial.println("Error4");
    }
  
    error = i2c_write(0x1C);
    if(error)
    {
       Serial.println("Error5");
    }
  
    error = i2c_write(0b00001000);
    if(error)
    {
       Serial.println("Error6");
    }
    i2c_stop();
    Serial.println("Setup finished");
  }
}

void loop() 
{
  {
    int error = i2c_start(MPU_addr<<1);
    if(error)
    {
       Serial.println("ErrorA");
    }

    error = i2c_write(0x3B);
    if(error)
    {
       Serial.println("ErrorB");
    }
    i2c_stop();
  }


  {
    uint8_t data[14];
    int error = i2c_receive(MPU_addr<<1,data,14);
    if(error)
    {
       Serial.println("ErrorC");
    }

    AcX = (data[0]<<8) | data[1];
    AcY = (data[2]<<8) | data[3];
    AcZ = (data[4]<<8) | data[5];
    Tmp = (data[6]<<8) | data[7];
    GyX = (data[8]<<8) | data[9];
    GyY = (data[10]<<8) | data[11];
    GyZ = (data[12]<<8) | data[13];
  }



  x= RAD_TO_DEG * (atan2(AcY, AcZ));
  y= RAD_TO_DEG * (atan2(AcX, AcZ));
  z= RAD_TO_DEG * (atan2(AcY, AcX));
  

  
  /*Serial.print("AcX= ");
  Serial.println(AcX);
  Serial.print("AcY= ");
  Serial.println(AcY);
  Serial.print("AcZ= ");
  Serial.println(AcZ);*/
  

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
