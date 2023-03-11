#ifndef sensor_hub
#define sensor_hub

#include <SPI.h>
#include "Arduino.h"
#include "Drivers/BMP390/BMP390_ESP32.h"
#include "Drivers/Battery Voltage/Battery_Voltage_Sensor.h"
#include "Drivers/LSM6DSOX/LSM6DSOX_ESP32.h"
#include "SensorData.h"
#include "../constants.h"
#include "elapsedMillis.h"


class SensorHub{
  public:
    SensorHub();

    // Returns 0 on success, -1 otherwise
    int begin();

    // This method should be called in a forever loop
    void update();

    // Stores sensor data
    SensorData data;

  private:
    // Available sensors
    BMP390 _barometer = BMP390(&_spi,Pins::SDI,Pins::SDO,Pins::SCK,Pins::BARO_CS);
    LSM6DSOX _imu = LSM6DSOX(&_spi,Pins::SDI,Pins::SDO,Pins::SCK,Pins::IMU_CS);
    BatteryVoltageSensor _voltage_sensor = BatteryVoltageSensor(Pins::BATTERY_VOLTAGE);

    // SPI object
    SPIClass _spi = SPIClass(HSPI);

    // Methods which update data with new sensor data
    void _update_imu();
    void _update_barometer();
    void _update_voltage();

    // Variable to store the intial barometer pressure
    float _initial_pressure;
    void _record_initial_barometer_pressure();

    // Timers for reading from sensors
    elapsedMicros _barometer_update_timer;
    elapsedMicros _imu_update_timer;
    elapsedMicros _voltage_sensor_update_timer;

    // Periods with which sensors should update
    const uint32_t _BAROMETER_UPDATE_PERIOD = 20000;  // 20 ms
    const uint32_t _IMU_UPDATE_PERIOD = 1000;  // 1 ms
    const uint32_t _VOLTAGE_SENSOR_UPDATE_PERIOD = 20000;  // 20 ms
};


#endif
