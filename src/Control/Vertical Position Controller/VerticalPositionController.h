#ifndef ESP32_DRONE_VERTICAL_POSITION_CONTROLLER
#define ESP32_DRONE_VERTICAL_POSITION_CONTROLLER

#include "../utils/PID Controller/PIDController.h"
#include "../../State Estimator/State.h"

// Proportional position control
class VerticalPositionController{
    public:
        VerticalPositionController(float kp);

        void begin();

        // Returns the controller output
        float update(float ref, const State& state);

        void reset();
    
    private:
        PIDController _vertical_position_controller;
};





#endif