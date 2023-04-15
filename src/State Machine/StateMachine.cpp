#include "StateMachine.h"

void StateMachine::begin(){
    _bluetooth->begin();
    _update_timer = 0;
    _battery_message_timer = 0;
}

void StateMachine::update(const SensorData& sensor_data, const State& state){
    if(_battery_message_timer > _BATTERY_MESSAGE_SEND_PERIOD_MS && _bluetooth_connected){
        _battery_message_timer = 0;
        _bluetooth->send_battery_percent(_battery_percent_remaining);
    }

    if(_update_timer > _UPDATE_PERIOD_MS){
        _update_timer = 0;

        // Updating battery voltage
        float battery_voltage = sensor_data.battery_voltage;
        float battery_percent = 100*(battery_voltage - _MIN_BATTERY_VOLTAGE)/(_MAX_BATTERY_VOLTAGE - _MIN_BATTERY_VOLTAGE);
        if(battery_percent > 100){
            battery_percent = 100;
        }
        else if(battery_percent <= 0){
            battery_percent = 0;
        }

        _battery_percent_remaining = (uint8_t) battery_percent;

        // Updating state variables
        if(fabsf(state.pitch) > _MAX_TILT_RAD || fabsf(state.roll) > _MAX_TILT_RAD){
            _crashed = true;
        }

        if(_battery_percent_remaining <= _LOW_BATTERY_LEVEL){
            _low_battery = true;
        }
        else{
            _low_battery = false;
        }

        if(_battery_percent_remaining <= _OUT_OF_BATTERY_LEVEL){
            _out_of_battery = true;
        }
        else{
            _out_of_battery = false;
        }

        _bluetooth_connected = _bluetooth->is_connected();

        bool _commanded_to_arm = _bluetooth->system_armed();

        // Logic for determining system arm state
        if(!_commanded_to_arm){
            _armed = false;
        }
        else if(_out_of_battery){
            _armed = false;
        }
        else if(_crashed){
            _armed = false;
        }
        else if(!_bluetooth_connected){
            _armed = false;
        }
        else if(_low_battery && !_armed){
            _armed = false;
        }
        else{
            _armed = true;
        }
    }
}