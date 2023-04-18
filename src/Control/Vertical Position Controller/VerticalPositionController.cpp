#include "VerticalPositionController.h"

VerticalPositionController::VerticalPositionController(float kp) :
    _vertical_position_controller(kp, 0, 0){}

void VerticalPositionController::begin(){
    _vertical_position_controller.begin();
}

float VerticalPositionController::update(float ref, const State& state){
    return _vertical_position_controller.update(ref, state.pos_z);
}

void VerticalPositionController::reset(){
    _vertical_position_controller.reset();
}
