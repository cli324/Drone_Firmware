#ifndef drone_constants
#define drone_constants

// Pinout
struct Pins{
  static const int SDI = 13;
  static const int SDO = 14;
  static const int SCK = 4;
  static const int MAG_CS = 27;
  static const int MAG_INT = 20;
  static const int IMU_CS = 26;
  static const int IMU_INT1 = 34;
  static const int IMUT_INT2 = 39;
  static const int BARO_CS = 25;
  static const int BARO_INT = 35;
  static const int BATTERY_VOLTAGE = 38;
  
  static const int FRONT_LEFT_MOTOR = 8;
  static const int FRONT_RIGHT_MOTOR = 33;
  static const int BACK_LEFT_MOTOR = 21;
  static const int BACK_RIGHT_MOTOR = 32;

  static const int LED = 2;
  static const int SWITCH = 0;
};

// Acceleration due to gravity
const float g = 9.8;


#endif
