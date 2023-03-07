#ifndef motor_hub
#define motor_hub

#include "Arduino.h"
#include "../constants.h"

// Interface for the motors
class MotorHub{
  public:
    MotorHub() = default;

    void begin(){
      ledcSetup(_front_left_channel,_pwm_frequency,_resolution);
      ledcSetup(_front_right_channel,_pwm_frequency,_resolution);
      ledcSetup(_back_left_channel,_pwm_frequency,_resolution);
      ledcSetup(_back_right_channel,_pwm_frequency,_resolution);
    
      ledcAttachPin(Pins::FRONT_LEFT_MOTOR,_front_left_channel);
      ledcAttachPin(Pins::FRONT_RIGHT_MOTOR,_front_right_channel);
      ledcAttachPin(Pins::BACK_LEFT_MOTOR,_back_left_channel);
      ledcAttachPin(Pins::BACK_RIGHT_MOTOR,_back_right_channel);
    
      ledcWrite(_front_left_channel,0);
      ledcWrite(_front_right_channel,0);
      ledcWrite(_back_right_channel,0);
      ledcWrite(_back_left_channel,0);
    }

    void set(float front_left_signal,float front_right_signal, float back_right_signal, float back_left_signal){
      _write_channel(front_left_signal,_front_left_channel);
      _write_channel(front_right_signal,_front_right_channel);
      _write_channel(back_right_signal,_back_right_channel);
      _write_channel(back_left_signal,_back_left_channel);
    }

  private:
    const uint8_t _front_left_channel = 0;
    const uint8_t _front_right_channel = 1;
    const uint8_t _back_right_channel = 2;
    const uint8_t _back_left_channel = 3;
    
    const uint16_t _pwm_frequency = 4000;
    const uint8_t _resolution = 10;

    void _write_channel(float value, uint8_t channel){
      if(value > 1 || value < 0){
        ledcWrite(channel,0);
      }
      else{
        int signal = (int) (value * (powf(2,(float)_resolution) - 1));
        ledcWrite(channel,signal);
      }
    }
};






#endif
