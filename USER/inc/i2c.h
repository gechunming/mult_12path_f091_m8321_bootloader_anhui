#ifndef __I2C_H__
#define __I2C_H__

int i2c_start();
void i2c_stop();
int  i2c_write_byte(unsigned char b);
unsigned char i2c_read_byte_nack();
unsigned char i2c_read_byte();
void i2c_init();

#endif
