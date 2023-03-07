#include "BMP390_ESP32.h"

BMP390::BMP390(SPIClass* spi,const int sdi, const int sdo, const int sck, const int cs):
  _spi(spi),_sdi(sdi),_sdo(sdo),_sck(sck),_cs(cs) {
  pinMode(_cs,OUTPUT);
  digitalWrite(_cs,HIGH);
}

void BMP390::begin(){
  // Starting up the SPI bus
  _spi->begin(_sck,_sdo,_sdi);
  delay(50);

  // Disabling FIFO
  write_register(FIFO_CONFIG_1,0);
  delay(20);

  // Set normal mode
  write_register(PWR_CTRL,0b00110011);
  delay(20);

  // Set oversampling rate to 8x pressure, 1x temperature
  write_register(OSR,0b00000011);
  delay(20);

  // Set ODR to 20 ms period
  write_register(ODR,0x02);
  delay(20);

  // Set IIR coefficient to 1
  write_register(CONFIG,0b00000010);
  delay(20);

  // Reading calibration data
  read_calibration_data();
}

float BMP390::read(){
  // Reading raw data from registers
  _spi->beginTransaction(_settings);
  digitalWrite(_cs,LOW);
  _spi->transfer(DATA_0 | READ_MASK);
  _spi->transfer(0);  // Dummy transfer
  uint32_t pressure_lower = _spi->transfer(0);
  uint32_t pressure_middle = _spi->transfer(0);
  uint32_t pressure_upper = _spi->transfer(0);
  uint32_t temperature_lower = _spi->transfer(0);
  uint32_t temperature_middle = _spi->transfer(0);
  uint32_t temperature_upper = _spi->transfer(0);
  digitalWrite(_cs,HIGH);
  _spi->endTransaction();

  uint32_t pressure = (pressure_upper << 16) | ((pressure_middle << 8) | pressure_lower);
  uint32_t temperature = (temperature_upper << 16) | ((temperature_middle << 8) | temperature_lower);

  // Converting data from raw adc value to pascals and degrees C via calibration coefficients
  BMP390_compensate_temperature(temperature);
  return BMP390_compensate_pressure(pressure);
}

bool BMP390::responding(){
  // Reading from product_id register
  _spi->beginTransaction(_settings);
  digitalWrite(_cs,LOW);
  _spi->transfer(CHIP_ID | READ_MASK);
  _spi->transfer(0);  // Dummy transfer
  int8_t chip_id = _spi->transfer(0);
  digitalWrite(_cs,HIGH);
  _spi->endTransaction();

  if(chip_id == _chip_id){
    return true;
  }
  else{
    return false;
  }
}


void BMP390::write_register(int8_t register_address,int8_t value){
  _spi->beginTransaction(_settings);
  digitalWrite(_cs,LOW);
  _spi->transfer(register_address & WRITE_MASK);
  _spi->transfer(value);
  digitalWrite(_cs,HIGH);
  _spi->endTransaction();
}

void BMP390::read_calibration_data(){
  _spi->beginTransaction(_settings);
  digitalWrite(_cs,LOW);
  _spi->transfer(NVM_PAR_T1 | READ_MASK);
  _spi->transfer(0);  // Dummy transfer
  uint16_t nvm_par_t1_lower = _spi->transfer(0);
  uint16_t nvm_par_t1_upper = _spi->transfer(0);
  uint16_t nvm_par_t2_lower = _spi->transfer(0);
  uint16_t nvm_par_t2_upper = _spi->transfer(0);
  int8_t nvm_par_t3 = _spi->transfer(0);
  int16_t nvm_par_p1_lower = _spi->transfer(0);
  int16_t nvm_par_p1_upper = _spi->transfer(0);
  int16_t nvm_par_p2_lower = _spi->transfer(0);
  int16_t nvm_par_p2_upper = _spi->transfer(0);
  int8_t nvm_par_p3 = _spi->transfer(0);
  int8_t nvm_par_p4 = _spi->transfer(0);
  uint16_t nvm_par_p5_lower = _spi->transfer(0);
  uint16_t nvm_par_p5_upper = _spi->transfer(0);
  uint16_t nvm_par_p6_lower = _spi->transfer(0);
  uint16_t nvm_par_p6_upper = _spi->transfer(0);
  int8_t nvm_par_p7 = _spi->transfer(0);
  int8_t nvm_par_p8 = _spi->transfer(0);
  int16_t nvm_par_p9_lower = _spi->transfer(0);
  int16_t nvm_par_p9_upper = _spi->transfer(0);
  int8_t nvm_par_p10 = _spi->transfer(0);
  int8_t nvm_par_p11 = _spi->transfer(0);
  digitalWrite(_cs,HIGH);
  _spi->endTransaction();

  // Storing calibration coefficients - scaling performed according to the datasheet
  uint16_t nvm_par_t1 = (nvm_par_t1_upper << 8) | nvm_par_t1_lower;
  calib_data.par_t1 = ((float)nvm_par_t1) / scalars.par_t1;
  
  uint16_t nvm_par_t2 = (nvm_par_t2_upper << 8) | nvm_par_t2_lower;
  calib_data.par_t2 = ((float)nvm_par_t2) / scalars.par_t2;
  
  calib_data.par_t3 = ((float)nvm_par_t3) / scalars.par_t3;
  
  int16_t nvm_par_p1 = (nvm_par_p1_upper << 8) | nvm_par_p1_lower;
  calib_data.par_p1 = ((float)(nvm_par_p1 - 16384)) / scalars.par_p1;

  int16_t nvm_par_p2 = (nvm_par_p2_upper << 8) | nvm_par_p2_lower;
  calib_data.par_p2 = ((float)(nvm_par_p2 - 16384)) / scalars.par_p2;

  calib_data.par_p3 = ((float)nvm_par_p3) / scalars.par_p3;

  calib_data.par_p4 = ((float)nvm_par_p4) / scalars.par_p4;

  uint16_t nvm_par_p5 = (nvm_par_p5_upper << 8) | nvm_par_p5_lower;
  calib_data.par_p5 = ((float)nvm_par_p5) / scalars.par_p5;

  uint16_t nvm_par_p6 = (nvm_par_p6_upper << 8) | nvm_par_p6_lower;
  calib_data.par_p6 = ((float)nvm_par_p6) / scalars.par_p6;

  calib_data.par_p7 = ((float)nvm_par_p7) / scalars.par_p7;

  calib_data.par_p8 = ((float)nvm_par_p8) / scalars.par_p8;

  int16_t nvm_par_p9 = (nvm_par_p9_upper << 8) | nvm_par_p9_lower;
  calib_data.par_p9 = ((float)nvm_par_p9) / scalars.par_p9;

  calib_data.par_p10 = ((float)nvm_par_p10) / scalars.par_p10;

  calib_data.par_p11 = ((float)nvm_par_p11) / scalars.par_p11;  
}

float BMP390::BMP390_compensate_temperature(uint32_t uncomp_temp){
  float partial_data1;
  float partial_data2;
  partial_data1 = ((float)uncomp_temp - calib_data.par_t1);
  partial_data2 = (float)(partial_data1 * calib_data.par_t2);
  /* Update the compensated temperature in calib structure since this is
  * needed for pressure calculation */
  calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;
  /* Returns compensated temperature */
  return calib_data.t_lin;
}

float BMP390::BMP390_compensate_pressure(uint32_t uncomp_press){
  /* Variable to store the compensated pressure */
  float comp_press;
  /* Temporary variables used for compensation */
  float partial_data1;
  float partial_data2;
  float partial_data3;
  float partial_data4;
  float partial_out1;
  float partial_out2;
  /* Calibration data */
  partial_data1 = calib_data.par_p6 * calib_data.t_lin;
  partial_data2 = calib_data.par_p7 * (calib_data.t_lin * calib_data.t_lin);
  partial_data3 = calib_data.par_p8 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
  partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = calib_data.par_p2 * calib_data.t_lin;
  partial_data2 = calib_data.par_p3 * (calib_data.t_lin * calib_data.t_lin);
  partial_data3 = calib_data.par_p4 * (calib_data.t_lin * calib_data.t_lin * calib_data.t_lin);
  partial_out2 = (float)uncomp_press *
  (calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = (float)uncomp_press * (float)uncomp_press;
  partial_data2 = calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data.par_p11;
  comp_press = partial_out1 + partial_out2 + partial_data4;
  return comp_press;
}
