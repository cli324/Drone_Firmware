#include "FirstOrderLPF.h"

FirstOrderLPF::FirstOrderLPF(float cutoff_frequency_hz):_w0(2*PI*cutoff_frequency_hz){}

FirstOrderLPF::FirstOrderLPF(float cutoff_frequency_hz, float initial_value):
    _w0(2*PI*cutoff_frequency_hz), _prev(initial_value), _initial_value_received(true){}

void FirstOrderLPF::begin(){
    _timer = 0;
}

void FirstOrderLPF::reset(){
    _timer = 0;
    _initial_value_received = false;
}

float FirstOrderLPF::filter(float value){
    if(_initial_value_received){
        float delta_T = (float)_timer / 1000000.0f;  // Time since last filtering in seconds
        // Resetting timer
        _timer = 0;

        float output = (delta_T * _w0 * value + _prev) / (1 + delta_T * _w0);
        _prev = output;
        return output;
    }
    else{
        _timer = 0;
        _initial_value_received = true;
        _prev = value;
        return value;
    }
}