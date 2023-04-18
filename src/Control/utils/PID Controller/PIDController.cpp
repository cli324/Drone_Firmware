#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float derivative_lpf_cutoff) :
    _kp(kp), _ki(ki), _kd(kd), _derivative_lpf(derivative_lpf_cutoff) {
        if(derivative_lpf_cutoff > 0){
            _filter_derivative = true;
        }
        
}

float PIDController::update(float ref, float feedback){
    float microseconds_in_seconds = 1000000;
    float delta_t_seconds = ((float) _timer) / microseconds_in_seconds;
    _timer = 0;

    return _proportional(ref, feedback) + _integral(ref, feedback, delta_t_seconds) - _derivative(feedback, delta_t_seconds);
}

void PIDController::begin(){
    _timer = 0;
    if(_filter_derivative){
        _derivative_lpf.begin();
    }
}

void PIDController::reset(){
    _error_integral = 0;
    _derivative_lpf.reset();
    _first_signal_received = false;
    _timer = 0;
}

float PIDController::_proportional(float ref, float feedback){
    return _kp * (ref - feedback);
}

float PIDController::_integral(float ref, float feedback, float delta_t_seconds){
    _error_integral += (ref - feedback) * delta_t_seconds;
    return _ki * _error_integral;
}

float PIDController::_derivative(float feedback, float delta_t_seconds){
    if(!_first_signal_received){
        _signal_prev = feedback;
        _first_signal_received = true;
        return 0.0f;
    }

    float derivative = (feedback - _signal_prev) / delta_t_seconds;
    _signal_prev = feedback;

    if(_filter_derivative){
        return _derivative_lpf.filter(_kd * derivative);
    }
    else{
        return _kd * derivative;
    }
}
