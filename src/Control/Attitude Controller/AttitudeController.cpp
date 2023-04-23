#include "AttitudeController.h"

AttitudeController::AttitudeController(float kp) :
    _roll_controller(kp, 0, 0),
    _pitch_controller(kp, 0, 0){}

void AttitudeController::begin(){
    _roll_controller.begin();
    _pitch_controller.begin();
}

void AttitudeController::update(float output[2], float refs[2], const State& state){
    float roll = refs[0]; float pitch = refs[1];
    output[0] = _roll_controller.update(roll, state.roll);
    output[1] = _pitch_controller.update(pitch, state.pitch);
}

void AttitudeController::reset(){
    _roll_controller.reset();
    _pitch_controller.reset();
}