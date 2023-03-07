#ifndef battery_voltage_sensor
#define battery_voltage_sensor


// Used to obtain the voltage of the lipo battery on the drone
// The voltage sensor is just an analog reading of the voltage from a voltage divider
// The values of the resistors in the voltage divider are 5.1k and 15k
class BatteryVoltageSensor{
  public:
    BatteryVoltageSensor(const int pin):_pin(pin){}

    void begin(){
      pinMode(_pin,INPUT);
    }

    float read(){
      // Raw ADC reading
      float reading = (float)analogRead(_pin);
      
      // Converting ADC reading to a voltage
      float ADC_max = 4095; float io_voltage = 3.3;
      float divider_voltage = io_voltage*reading/ADC_max;

      // Converting voltage from the voltage divider to the battery voltage
      float R1 = 5.1; float R2 = 15;
      float battery_voltage = divider_voltage*(R1+R2)/R2;

      return battery_voltage;
    }

  private:
    const int _pin;
};



#endif
