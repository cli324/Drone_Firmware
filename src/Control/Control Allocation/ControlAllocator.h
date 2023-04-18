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
        void compute_motor_setpoints(float motor_setpoints[4], float controller_outputs[4]);


    private:
        // -----------------------------------System-specific parameters----------------------------------------
        const float _MIN_ALLOWED_MOTOR_SETPOINT = 0.03;

        /* 
        Matrix which relates angular rates and thrust to motor force commands
        This matrix was obtained by writing out the 3 angular momentum balance equations along with 1 linear momentum balance
        equation for thrust in the body-fixed z direction.
        */

       BLA::Matrix<4,4> _control_allocation_matrix = {
        0.0008    ,  0.00089333,  0.00010785,  0.03666667,
        0.0008    , -0.00089333, -0.00010785,  0.03666667,
        -0.0008    , -0.00089333,  0.00010785,  0.03666667,
        -0.0008    ,  0.00089333, -0.00010785,  0.03666667
       };
};


#endif