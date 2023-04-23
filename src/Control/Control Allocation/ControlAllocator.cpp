#include "ControlAllocator.h"

void ControlAllocator::compute_motor_setpoints(float motor_setpoints[4], float controller_outputs[4], const State& state){
    float roll_accel_setpoint = controller_outputs[0];
    float pitch_accel_setpoint = controller_outputs[1];
    float yaw_accel_setpoint = controller_outputs[2];
    float vertical_accel_setpoint = controller_outputs[3];

    // Converting the vertical acceleration setpoint to a thrust setpoint
    float thrust_setpoint = _vertical_acceleration_to_thrust(vertical_accel_setpoint, state);
    BLA::Matrix<4> setpoints = {pitch_accel_setpoint,roll_accel_setpoint,yaw_accel_setpoint,thrust_setpoint};

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
    else if(largest_motor_setpoint / scale_factor > 1){
        offset = -(largest_motor_setpoint/scale_factor) + 1;
    }

    for(uint8_t i=0; i<motors_count; i++){
        float setpoint = (motor_commands_raw(i) / scale_factor) + offset;
        if(setpoint < _MIN_ALLOWED_MOTOR_SETPOINT){
            setpoint = _MIN_ALLOWED_MOTOR_SETPOINT;
        }
        motor_setpoints[i] = setpoint;
    }
}


float ControlAllocator::_vertical_acceleration_to_thrust(float accel, const State& state){
    float pitch = state.pitch; float roll = state.roll;
    if(pitch > _MAX_TILT_COMPENSATION){
        pitch = _MAX_TILT_COMPENSATION;
    }
    else if(pitch < -_MAX_TILT_COMPENSATION){
        pitch = -_MAX_TILT_COMPENSATION;
    }

    if(roll > _MAX_TILT_COMPENSATION){
        roll = _MAX_TILT_COMPENSATION;
    }
    else if(roll < _MAX_TILT_COMPENSATION){
        roll = -_MAX_TILT_COMPENSATION;
    }

    // Calculating the real component of the quaternion that describes the attitude of the drone
    // See the link for additional information on the calculation: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion
    float omega = cosf(state.pitch / 2.0) * cosf(state.roll / 2.0);

    // Calculating angle in the axis-angle representation of the drone orientation
    float theta = 2 * acosf(omega);

    float thrust = (m*(accel + g))/cosf(theta);
    return thrust;
}
