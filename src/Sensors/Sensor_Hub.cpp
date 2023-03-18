#include "Sensor_Hub.h"

SensorHub::SensorHub(){
  // Writing magnetometer chip select high - Sensor not currently used
  pinMode(Pins::MAG_CS,OUTPUT); digitalWrite(Pins::MAG_CS,HIGH);
}


int SensorHub::begin(){
  _barometer.begin();
  _imu.begin();
  _voltage_sensor.begin();

  bool sensors_good = _imu.responding() && _barometer.responding();

  if(sensors_good){
    _update_voltage();
    _update_imu();
    _record_initial_barometer_pressure();
    _update_barometer();

    _barometer_update_timer = 0;
    _imu_update_timer = 0;
    _voltage_sensor_update_timer = 0;
  }
  else{
    return -1;
  }
}


void SensorHub::update(){
  if(_imu_update_timer >= _IMU_UPDATE_PERIOD){
    _update_imu();
    _imu_update_timer -= _IMU_UPDATE_PERIOD;
  }

  if(_barometer_update_timer >= _BAROMETER_UPDATE_PERIOD){
    _update_barometer();
    _barometer_update_timer -= _BAROMETER_UPDATE_PERIOD;
  }

  if(_voltage_sensor_update_timer >= _VOLTAGE_SENSOR_UPDATE_PERIOD){
    _update_voltage();
    _voltage_sensor_update_timer -= _VOLTAGE_SENSOR_UPDATE_PERIOD;
  }
}


void SensorHub::_record_initial_barometer_pressure(){
  // Average 10 sensor readings to obtain the initial barometric pressure
  int number_of_readings = 10;
  for(int i=0; i<number_of_readings; i++){
    float pressure_reading = _barometer.read();
    _initial_pressure += pressure_reading / (float) number_of_readings;
    delayMicroseconds(_BAROMETER_UPDATE_PERIOD);
  }
}


void SensorHub::_update_imu(){
  IMUData imu_data;
  _imu.read(imu_data);

  // Sign changes to compensate for orientation of IMU on the PCB
  // Body fixed frame is right, front, up
  data.accel_x = imu_data.accX;
  data.accel_y = imu_data.accY;
  data.accel_z = -imu_data.accZ;
  data.w_x = imu_data.wX;
  data.w_y = imu_data.wY;
  data.w_z = -imu_data.wZ;

  data.imu_updated = true;
}


void SensorHub::_update_barometer(){
  float pressure = _barometer.read();

  // Converting pressure reading to a relative change in altitude from the starting position
  float delta_p = pressure - _initial_pressure;
  // Assuming density of air to be about 1.19 kg/m^3
  // Change in height = -(Change in pressure) / (g * density_of_air)
  float density_of_air = 1.19;
  data.pos_z = -delta_p / (g * density_of_air);
  
  data.barometer_updated = true;
}


void SensorHub::_update_voltage(){
  data.battery_voltage = _voltage_sensor.read();

  data.voltage_updated = true;
}
