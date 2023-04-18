#include "Controls.h"

void Controls::begin(){
    _rate_controller.begin();
    _attitude_controller.begin();
    _vertical_velocity_controller.begin();
    _vertical_position_controller.begin();

    _rate_controller_update_timer = 0;
    _attitude_controller_update_timer = 0;
    _position_and_velocity_controller_update_timer = 0;
    _thrust_integration_timer = 0;
}


void Controls::update(bool system_armed, const State& state){
    if(!system_armed && _controllers_running){
        // Disarm signal was just sent
        _controllers_running = false;
        _reset_controllers();
        _new_motor_setpoint_available = true;
        return;
    }
    else if(!system_armed){
        return;
    }
    else if(system_armed && !_controllers_running){
        // System arm signal just sent
        _controllers_running = true;
        _reset_controllers();
        return;
    }

    // Controller updates
    if(_position_and_velocity_controller_update_timer >= _POSITION_AND_VELOCITY_CONTROLLER_UPDATE_PERIOD_US){
        _position_and_velocity_controller_update_timer -= _POSITION_AND_VELOCITY_CONTROLLER_UPDATE_PERIOD_US;
        _velocity_setpoint = _vertical_position_controller.update(_position_setpoint,state);
        _acceleration_setpoint = _vertical_velocity_controller.update(_velocity_setpoint, state);
    }

    if(_attitude_controller_update_timer >= _ATTITUDE_CONTROLLER_UPDATE_PERIOD_US){
        _attitude_controller_update_timer -= _ATTITUDE_CONTROLLER_UPDATE_PERIOD_US;
        float roll_pitch_rate_setpoints[2];
        _attitude_controller.update(roll_pitch_rate_setpoints,_attitude_setpoints,state);
        _rate_setpoints[0] = roll_pitch_rate_setpoints[0];
        _rate_setpoints[1] = roll_pitch_rate_setpoints[1];
    }

    if(_rate_controller_update_timer >= _RATE_CONTROLLER_UPDATE_PERIOD_US){
        _rate_controller_update_timer -= _RATE_CONTROLLER_UPDATE_PERIOD_US;
        float angular_accels[3];
        _rate_controller.update(angular_accels,_rate_setpoints,state);

        float controller_setpoints[4];

        controller_setpoints[0] = angular_accels[0];
        controller_setpoints[1] = angular_accels[1];
        controller_setpoints[2] = angular_accels[2];
        controller_setpoints[3] = _acceleration_setpoint;

        _control_allocator.compute_motor_setpoints(_motor_setpoints,controller_setpoints);
        _new_motor_setpoint_available = true;
    }
}


void Controls::update_joystick_signals(bool system_armed, float joystick_commands[4]){
    if(system_armed){
        float thrust_command = _adjust_for_deadband(joystick_commands[0]);
        float yaw_command = _adjust_for_deadband(joystick_commands[1]);
        float pitch_command = _adjust_for_deadband(joystick_commands[2]);
        float roll_command = _adjust_for_deadband(joystick_commands[3]);

        _rate_setpoints[2] = yaw_command * _max_yaw_rate;
        _attitude_setpoints[0] = roll_command * _max_tilt;
        _attitude_setpoints[1] = pitch_command * _max_tilt;

        // Treating thrust command as a velocity setpoint
        // Integrating the velocity setpoint to obtain a position setpoint
        // Could instead feed the velocity setpoint directly to the velocity controller, but would have to figure out how to switch
        // between position and velocity setpoints.
        float delta_t = (float)(_thrust_integration_timer) / 1000.0f;
        _thrust_integration_timer = 0;

        _position_setpoint += thrust_command * _max_vertical_velocity * delta_t;
    }
}

void Controls::get_motor_setpoints(float setpoints[4]){
    memcpy(setpoints,_motor_setpoints,sizeof(_motor_setpoints));
    _new_motor_setpoint_available = false;
}


float Controls::_adjust_for_deadband(float command){
    if(command > _joysticks_deadband){
        return command - _joysticks_deadband;
    }
    else if(command < -_joysticks_deadband){
        return command + _joysticks_deadband;
    }
    else{
        return 0.0;
    }
}

void Controls::_reset_controllers(){
    _clear_array(_motor_setpoints,sizeof(_motor_setpoints)/sizeof(_motor_setpoints[0]));
    _clear_array(_rate_setpoints,sizeof(_rate_setpoints)/sizeof(_rate_setpoints[0]));
    _clear_array(_attitude_setpoints,sizeof(_attitude_setpoints)/sizeof(_attitude_setpoints[0]));
    _acceleration_setpoint = 0;
    _velocity_setpoint = 0;
    _position_setpoint = 0;

    _rate_controller.reset();
    _attitude_controller.reset();
    _vertical_velocity_controller.reset();
    _vertical_position_controller.reset();

    _rate_controller_update_timer = 0;
    _attitude_controller_update_timer = 0;
    _position_and_velocity_controller_update_timer = 0;
    _thrust_integration_timer = 0;
}

void Controls::_clear_array(float arr[], uint8_t length){
    for(uint8_t i=0; i<length; i++){
        arr[i] = 0;
    }
}
