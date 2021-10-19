#ifndef i2c_h
#define i2c_h

#include <util/twi.h>

//i2c from https://github.com/knightshrub/I2C-master-lib/blob/master/i2c_master.c
#ifndef  F_CPU
#define F_CPU 16000000UL
#endif
#define F_SCL 100000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

void i2c_init(void);

uint8_t i2c_start(uint8_t address);

uint8_t i2c_write(uint8_t data);

void i2c_stop(void);

uint8_t i2c_read_ack(void);

uint8_t i2c_read_nack(void);

uint8_t i2c_receive(uint8_t address, uint8_t* data, uint16_t length);

#endif