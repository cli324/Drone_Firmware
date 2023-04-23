#include "RateController.h"

RateController::RateController(float pitch_roll_kp, float pitch_roll_ki, float pitch_roll_kd, float yaw_kp, float yaw_ki, float yaw_kd) :
    _pitch_rate_controller(pitch_roll_kp, pitch_roll_ki, pitch_roll_kd, _derivative_cutoff_frequency),
    _roll_rate_controller(pitch_roll_kp, pitch_roll_ki, pitch_roll_kd, _derivative_cutoff_frequency),
    _yaw_rate_controller(yaw_kp, yaw_ki, yaw_kd, _derivative_cutoff_frequency) {}


void RateController::begin(){
    _pitch_rate_controller.begin();
    _roll_rate_controller.begin();
    _yaw_rate_controller.begin();
}

void RateController::update(float outputs[3], float refs[3], const State& state){
    float roll_rate = refs[0]; float pitch_rate = refs[1]; float yaw_rate = refs[2];
    outputs[0] = _roll_rate_controller.update(roll_rate, state.wy);
    outputs[1] = _pitch_rate_controller.update(pitch_rate, state.wx);
    outputs[2] = _yaw_rate_controller.update(yaw_rate, state.wz);
}

void RateController::reset(){
    _pitch_rate_controller.reset();
    _roll_rate_controller.reset();
    _yaw_rate_controller.reset();
}