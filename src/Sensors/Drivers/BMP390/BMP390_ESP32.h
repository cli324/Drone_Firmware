#ifndef BMP_390_ESP32
#define BMP_390_ESP32

#include "Arduino.h"
#include <SPI.h>

// Driver for the BMP390 Pressure sensor
// Datasheet: https://www.mouser.com/datasheet/2/783/bst_bmp390_ds002-2448819.pdf

class BMP390{
  public:
    BMP390(SPIClass* spi,const int sdi, const int sdo, const int sck, const int cs);

    void begin();

    // Returns a pressure reading with units of pascals
    float read();

    bool responding();

  private:
    // Struct to hold calibration coefficient scalars
    struct CalibrationCoefficientScalars{
      float par_t1 = 0.00390625f;  // 2^-8
      float par_t2 = 1073741824.0f;  // 2^30
      float par_t3 = 281474976710656.0f;  // 2^48
      float par_p1 = 1048576.0f;  // 2^20
      float par_p2 = 536870912.0f;  // 2^29
      float par_p3 = 4294967296.0f;  // 2^32
      float par_p4 = 137438953472.0f;  // 2^37
      float par_p5 = 0.125f;  // 2^-3
      float par_p6 = 64.0f;  // 2^6
      float par_p7 = 256.0f;  // 2^8
      float par_p8 = 32768.0f;  // 2^15
      float par_p9 = 281474976710656.0f;  // 2^48
      float par_p10 = 281474976710656.0f;  // 2^48
      float par_p11 = 36893488147419103000.0f;  // 2^65
    };
    const CalibrationCoefficientScalars scalars;

    struct CalibrationData{
      float par_t1;
      float par_t2;
      float par_t3;
      float par_p1;
      float par_p2;
      float par_p3;
      float par_p4;
      float par_p5;
      float par_p6;
      float par_p7;
      float par_p8;
      float par_p9;
      float par_p10;
      float par_p11;
      float t_lin;
    };
    CalibrationData calib_data;
  
    // SPI pins
    const int _sdi;
    const int _sdo;
    const int _sck;
    const int _cs;

    // SPI object
    SPIClass* _spi;
    
    //SPI settings
    SPISettings _settings = SPISettings(10000000,MSBFIRST,SPI_MODE0);

    // Method to write value to sensor register
    void write_register(int8_t register_address,int8_t value);

    // Method for reading calibration data
    void read_calibration_data();

    // Method for temperature compensation - taken from the datasheet
    float BMP390_compensate_temperature(uint32_t uncomp_temp);

    // Method for pressure compensation - taken from the datasheet
    float BMP390_compensate_pressure(uint32_t uncomp_press);

    const uint8_t _chip_id = 0b01100000;

    // Masks
    const uint8_t READ_MASK = 0b10000000;
    const uint8_t WRITE_MASK = 0b01111111;

    // Registers
    const uint8_t CHIP_ID = 0x00;
    const uint8_t DATA_0 = 0x04;  // Pressure register 1
    const uint8_t DATA_1 = 0x05;  // Pressure register 2
    const uint8_t DATA_2 = 0x06;  // Pressure register 3
    const uint8_t DATA_3 = 0x07;  // Temperature register 1
    const uint8_t DATA_4 = 0x08;  // Temperature register 2
    const uint8_t DATA_5 = 0x09;  // Temperature register 3
    const uint8_t FIFO_CONFIG_1 = 0x17;  // Enable/disable the FIFO
    const uint8_t PWR_CTRL = 0x1B;  // Enable/disable temperature and pressure measurement. Also can set measurement mode.
    const uint8_t OSR = 0x1C;
    const uint8_t ODR = 0x1D;
    const uint8_t CONFIG = 0x1F;
    const uint8_t NVM_PAR_T1 = 0x31;  // First calibration coefficient register
};







#endif
