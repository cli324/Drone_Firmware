#include "LSM6DSOX_ESP32.h"

LSM6DSOX::LSM6DSOX(SPIClass* spi,const int sdi, const int sdo, const int sck, const int cs):
  _spi(spi),_sdi(sdi),_sdo(sdo),_sck(sck),_cs(cs){
  // Digital I/O
  pinMode(_cs,OUTPUT);
  digitalWrite(_cs,HIGH);
}

void LSM6DSOX::begin(){
  // Starting up the SPI bus
  _spi->begin(_sck,_sdo,_sdi);
  delay(50);
  
  // Setting accelerometer settings
  write_register(CTRL1_XL,ACCEL_ODR_FS_SETTINGS);
  delayMicroseconds(5); // Delay needed because writes are to non-consecutive registers
  
  // Setting gyro settings
  write_register(CTRL2_G,GYRO_ODR_FS_SETTINGS);
  delayMicroseconds(5);
    
  // Disable FIFO
  write_register(FIFO_CTRL4,FIFO_DISABLE);

  delay(50);
}

void LSM6DSOX::write_register(int8_t register_address,int8_t value){
  _spi->beginTransaction(_settings);
  digitalWrite(_cs,LOW);
  _spi->transfer(register_address & WRITE_MASK);
  _spi->transfer(value);
  digitalWrite(_cs,HIGH);
  _spi->endTransaction();
}

void LSM6DSOX::read(IMUData& data){
  _spi->beginTransaction(_settings);
  digitalWrite(_cs,LOW);

  // Gyro data
  _spi->transfer(OUTX_L_G | READ_MASK);
  int16_t raw_gx_l = _spi->transfer(OUTX_H_G | READ_MASK);
  int16_t raw_gx_h = _spi->transfer(OUTY_L_G | READ_MASK);
  int16_t raw_gy_l = _spi->transfer(OUTY_H_G | READ_MASK);
  int16_t raw_gy_h = _spi->transfer(OUTZ_L_G | READ_MASK);
  int16_t raw_gz_l = _spi->transfer(OUTZ_H_G | READ_MASK);
  int16_t raw_gz_h = _spi->transfer(OUTX_L_A | READ_MASK);
  
  // Acceleration data
  int16_t raw_x_l = _spi->transfer(OUTX_H_A | READ_MASK);
  int16_t raw_x_h = _spi->transfer(OUTY_L_A | READ_MASK);
  int16_t raw_y_l = _spi->transfer(OUTY_H_A | READ_MASK);
  int16_t raw_y_h = _spi->transfer(OUTZ_L_A | READ_MASK);
  int16_t raw_z_l = _spi->transfer(OUTZ_H_A | READ_MASK);
  int16_t raw_z_h = _spi->transfer(OUTX_L_G | READ_MASK);

  digitalWrite(_cs,HIGH);
  _spi->endTransaction();

  float acc_x = register_values_to_data(raw_x_l,raw_x_h,ACCEL_SENSITIVITY);
  float acc_y = register_values_to_data(raw_y_l,raw_y_h,ACCEL_SENSITIVITY);
  float acc_z = register_values_to_data(raw_z_l,raw_z_h,ACCEL_SENSITIVITY);
  float gyr_x = register_values_to_data(raw_gx_l,raw_gx_h,GYRO_SENSITIVITY);
  float gyr_y = register_values_to_data(raw_gy_l,raw_gy_h,GYRO_SENSITIVITY);
  float gyr_z = register_values_to_data(raw_gz_l,raw_gz_h,GYRO_SENSITIVITY);

  data.accX = acc_x; data.accY = acc_y; data.accZ = acc_z;
  data.wX = gyr_x; data.wY = gyr_y; data.wZ = gyr_z;
}

float LSM6DSOX::register_values_to_data(int16_t register_lower,int16_t register_upper,float sensitivity){
  int16_t twos_complement = (register_upper << 8) | register_lower;
  int16_t raw_val = ~twos_complement + 1;
  float val = ((float)raw_val)*sensitivity;
  return val;
}

bool LSM6DSOX::responding(){
  _spi->beginTransaction(_settings);
  digitalWrite(_cs,LOW);
  _spi->transfer(WHO_AM_I | READ_MASK);
  int8_t default_name = _spi->transfer(WHO_AM_I | READ_MASK);
  digitalWrite(_cs,HIGH);
  _spi->endTransaction();

  if(default_name == PRODUCT_ID){
    return true;
  }
  else{
    return false;
  }
}
