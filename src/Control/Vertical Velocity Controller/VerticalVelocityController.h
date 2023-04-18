#ifndef ESP32_DRONE_VERTICAL_VELOCITY_CONTROLLER
#define ESP32_DRONE_VERTICAL_VELOCITY_CONTROLLER

#include "../utils/PID Controller/PIDController.h"
#include "../../State Estimator/State.h"

// Vertical velocity PID control
class VerticalVelocityController{
    public:
        VerticalVelocityController(float kp, float ki, float kd);

        void begin();

        // Return value is output from the controller
        float update(float ref, const State& state);

        void reset();
    
    private:
        PIDController _vertical_velocity_controller;
        
};





#endif