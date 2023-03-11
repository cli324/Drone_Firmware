#ifndef SENSOR_DATA
#define SENSOR_DATA

// Body fixed frame is right (x), front (y), up (z)
struct SensorData{
  float pos_z = 0;  // Height relative to initial height in meters
  float accel_x;  // Acceleration units are m/s^2
  float accel_y;
  float accel_z;
  float w_x;  // Angular rate units are rad/s
  float w_y;
  float w_z;
  float battery_voltage;  // Units are volts

  // Flags
  bool barometer_updated = false;
  bool imu_updated = false;
  bool voltage_updated = false;
};


#endif