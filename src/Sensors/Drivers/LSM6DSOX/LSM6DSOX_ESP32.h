#ifndef LSM6DSOX_ESP32
#define LSM6DSOX_ESP32

#include "Arduino.h"
#include <SPI.h>

// Struct to hold IMU sensor data
struct IMUData{
  float accX;
  float accY;
  float accZ;
  float wX;
  float wY;
  float wZ;
};

// Driver for the LSM6DSOX IMU
// Datasheet may be found here: https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
class LSM6DSOX{
  public:
    LSM6DSOX(SPIClass* spi,const int sdi, const int sdo, const int sck, const int cs);

    void begin();

    void read(IMUData& data);

    bool responding();

  private:
    SPIClass* _spi;
    
    // SPI pins
    const int _sdi;
    const int _sdo;
    const int _sck;
    const int _cs;

    // SPI settings
    SPISettings _settings = SPISettings(10000000,MSBFIRST,SPI_MODE0);

    // Method to write value to IMU register
    void write_register(int8_t register_address,int8_t value);

    // Method to convert raw register values to an acceleration or angular rate
    float register_values_to_data(int16_t register_lower,int16_t register_upper,float sensitivity);
    
    // IMU constants, register addresses, and register values
    const int8_t WRITE_MASK = 0b01111111;
    const int8_t READ_MASK = 0b10000000;
  
    const int8_t WHO_AM_I = 0xF;
    const int8_t CTRL1_XL = 0x10;
    const int8_t CTRL2_G = 0x11;
    const int8_t FIFO_CTRL4 = 0x0A;
    
    const int8_t OUTX_L_A = 0x28;
    const int8_t OUTX_H_A = 0x29;
    const int8_t OUTY_L_A = 0x2A;
    const int8_t OUTY_H_A = 0x2B;
    const int8_t OUTZ_L_A = 0x2C;
    const int8_t OUTZ_H_A = 0x2D;
  
    const int8_t OUTX_L_G = 0x22;
    const int8_t OUTX_H_G = 0x23;
    const int8_t OUTY_L_G = 0x24;
    const int8_t OUTY_H_G = 0x25;
    const int8_t OUTZ_L_G = 0x26;
    const int8_t OUTZ_H_G = 0x27;
  
    // Desired output data rate and FIFO settings
    const int8_t ACCEL_ODR_FS_SETTINGS = 0b10001100;
    const int8_t GYRO_ODR_FS_SETTINGS = 0b10001000;
    const int8_t FIFO_DISABLE = 0;
  
    // Constants from the datasheet
    const float g = 9.8;
    const float ACCEL_SENSITIVITY = g*0.244/1000.0;
    const float GYRO_SENSITIVITY = (35.0/1000.0)*PI/180.0;
    const float MAX_ACCEL = 8.0*g;
    const float MAX_GYRO = 1000.0*PI/180.0;
    const int8_t PRODUCT_ID = 0b01101100;
};





#endif
