#include "ControlAllocator.h"

void ControlAllocator::compute_motor_setpoints(float motor_setpoints[4], float controller_outputs[4]){
    BLA::Matrix<4> setpoints = {controller_outputs[0], controller_outputs[1], controller_outputs[2], controller_outputs[3]+g};

    BLA::Matrix<4> motor_commands_raw = _control_allocation_matrix * setpoints;

    const uint8_t motors_count = 4;

    // Adjusting motor setpoints so that they are in the range [_MIN_ALLOWED_MOTOR_SETPOINT, 1]
    float smallest_motor_setpoint = motor_commands_raw(0);
    float largest_motor_setpoint = smallest_motor_setpoint;
    for(uint8_t i=0; i<motors_count; i++){
        float motor_setpoint = motor_commands_raw(i);
        if(motor_setpoint > largest_motor_setpoint){
            largest_motor_setpoint = motor_setpoint;
        }
        else if(motor_setpoint < smallest_motor_setpoint){
            smallest_motor_setpoint = motor_setpoint;
        }
    }

    float scale_factor = 1;
    float offset = 0;
    if(largest_motor_setpoint - smallest_motor_setpoint > 1){
        scale_factor = largest_motor_setpoint - smallest_motor_setpoint;
    }
    if(smallest_motor_setpoint < 0){
        offset = fabsf(smallest_motor_setpoint);
    }

    for(uint8_t i=0; i<motors_count; i++){
        float setpoint = (motor_commands_raw(i) / scale_factor) + offset;
        if(setpoint < _MIN_ALLOWED_MOTOR_SETPOINT){
            setpoint = _MIN_ALLOWED_MOTOR_SETPOINT;
        }
        motor_setpoints[i] = setpoint;
    }
}