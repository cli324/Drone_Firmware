#include "VerticalVelocityController.h"

VerticalVelocityController::VerticalVelocityController(float kp, float ki, float kd) :
    _vertical_velocity_controller(kp, ki, kd, _derivative_lpf_cutoff){}

void VerticalVelocityController::begin(){
    _vertical_velocity_controller.begin();
}

float VerticalVelocityController::update(float ref, const State& state){
    return _vertical_velocity_controller.update(ref, state.vel_z);
}

void VerticalVelocityController::reset(){
    _vertical_velocity_controller.reset();
}