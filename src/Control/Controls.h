#ifndef ESP32_DRONE_CONTROLS
#define ESP32_DRONE_CONTROLS

#include "Rate Controller/RateController.h"
#include "Control Allocation/ControlAllocator.h"
#include "Attitude Controller/AttitudeController.h"
#include "Vertical Velocity Controller/VerticalVelocityController.h"
#include "Vertical Position Controller/VerticalPositionController.h"

#include "elapsedMillis.h"

/*
Main controller class for the drone. Control system is a cascaded PID control architecture similar to that used by PX4.
*/

class Controls{
    public:
        Controls() = default;

        void begin();

        // Main control loop logic
        void update(bool system_armed, const State& state);
        void update_joystick_signals(bool system_armed, float joystick_commands[4]);

        // Populates setpoints with front left, front right, back right, and back left motor setpoints (in that order)
        void get_motor_setpoints(float setpoints[4]);

        bool new_motor_setpoint_available(){return _new_motor_setpoint_available;}


    private:
        // -----Control gains-----
        // Rate controller
        const float _roll_pitch_rate_kp = 0;
        const float _roll_pitch_rate_ki = 0;
        const float _roll_pitch_rate_kd = 0;
        const float _yaw_rate_kp = 0;
        const float _yaw_rate_ki = 0;
        const float _yaw_rate_kd = 0;

        // Attitude controller
        const float _attitude_kp = 0;

        // Velocity controller
        const float _velocity_kp = 0;
        const float _velocity_ki = 0;
        const float _velocity_kd = 0;

        // Position Controller
        const float _position_kp = 0;

        // -----Controllers------
        ControlAllocator _control_allocator;
        RateController _rate_controller = 
            RateController(_roll_pitch_rate_kp, _roll_pitch_rate_ki, _roll_pitch_rate_kd, _yaw_rate_kp, _yaw_rate_ki, _yaw_rate_kd);
        AttitudeController _attitude_controller = AttitudeController(_attitude_kp);
        VerticalVelocityController _vertical_velocity_controller = VerticalVelocityController(_velocity_kp, _velocity_ki, _velocity_kd);
        VerticalPositionController _vertical_position_controller = VerticalPositionController(_position_kp);

        // ------Timers---------
        elapsedMicros _rate_controller_update_timer;
        elapsedMicros _attitude_controller_update_timer;
        elapsedMicros _position_and_velocity_controller_update_timer;

        elapsedMillis _thrust_integration_timer;

        const uint16_t _RATE_CONTROLLER_UPDATE_PERIOD_US = 1000;  // 1 khz
        const uint16_t _ATTITUDE_CONTROLLER_UPDATE_PERIOD_US = 4000;  // 250 hz
        const uint16_t _POSITION_AND_VELOCITY_CONTROLLER_UPDATE_PERIOD_US = 20000;  // 50 hz

        // ----Controller setpoints----
        float _motor_setpoints[4] = {0,0,0,0};
        float _rate_setpoints[3] = {0,0,0};
        float _attitude_setpoints[2] = {0,0};
        float _acceleration_setpoint = 0;
        float _velocity_setpoint = 0;
        float _position_setpoint = 0;

        // ----State variables for the class---
        bool _new_motor_setpoint_available = false;
        bool _controllers_running = false;

        const float _joysticks_deadband = 0.1;
        const float _max_tilt = 30.0*PI/180.0;  // rad
        const float _max_yaw_rate = PI;  // rad/s
        const float _max_vertical_velocity = 1.0;  // m/s


        // ------Miscellaneous--------
        float _adjust_for_deadband(float command);  // Helper function to adjust joystick commands based upon _joysticks_deadband
        void _clear_array(float arr[], uint8_t length);  // Fills the input array with 0s
        void _reset_controllers();

};







#endif