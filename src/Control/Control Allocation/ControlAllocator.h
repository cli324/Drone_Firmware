#ifndef ESP32_DRONE_CONTROL_ALLOCATOR
#define ESP32_DRONE_CONTROL_ALLOCATOR

#include "../../constants.h"
#include "../../State Estimator/State.h"
#include <BasicLinearAlgebra.h>

/*
This class converts angular acceleration and vertical acceleration setpoints to motor setpoints
*/
class ControlAllocator{
    public:
        ControlAllocator() = default;

        /*
        Method which converts force setpoints to motor commands (which are in the range [0,1])
        Force setpoints are made positive by adding the negative of the smallest negative force setpoint to all force setpoints
        The thrust setpoint is reduced if it would result in a motor command greater than 1
        Finally, motor setpoints smaller than the minimum allowed setpoint are set to the minimum (to prevent motors from ceasing to spin midflight)
        motor_setpoints is filled with the front_left, front_right, back_right, and back_left motor setpoints, in that order
        controller_outputs should contain the roll, pitch, and yaw rate controller setpoints as well as the vertical acceleration setpoint
        */
        void compute_motor_setpoints(float motor_setpoints[4], float controller_outputs[4], const State& state);


    private:
        // -----------------------------------System-specific parameters----------------------------------------
        const float _MIN_ALLOWED_MOTOR_SETPOINT = 0.03;

        // Converts a vertical acceleration command to a thrust command, compensating for the attitude of the aircraft
        // If the pitch or roll of the aircraft are greater than _MAX_TILT_COMPENSATION, then (+/-) _MAX_TILT_COMPENSATION is used instead
        // This is to guard against the case when a vertical acceleration command is unachievable due to the orientation of the aircraft
        float _vertical_acceleration_to_thrust(float accel, const State& state);
        const float _MAX_TILT_COMPENSATION = 60.0 * PI / 180.0;

        /* 
        Matrix which relates angular rates and thrust to motor force commands
        This matrix was obtained by writing out the 3 angular momentum balance equations along with 1 linear momentum balance
        equation for thrust in the body-fixed z direction.
        */

       BLA::Matrix<4,4> _control_allocation_matrix = {
        8.00000000E-04,  8.93333333E-04,  5.39266667E-03, 1.66666667E+00,
        8.00000000E-04, -8.93333333E-04, -5.39266667E-03, 1.66666667E+00,
        -8.00000000E-04, -8.93333333E-04,  5.39266667E-03, 1.66666667E+00,
       -8.00000000E-04,  8.93333333E-04, -5.39266667E-03, 1.66666667E+00
       };
};


#endif