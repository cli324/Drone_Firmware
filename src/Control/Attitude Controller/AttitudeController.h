#ifndef ESP32_DRONE_ATTITUDE_CONTROLLER
#define ESP32_DRONE_ATTITUDE_CONTROLLER

#include "../utils/PID Controller/PIDController.h"
#include "../../State Estimator/State.h"

// Proportional controller for roll and pitch
class AttitudeController{
    public:
        AttitudeController(float kp);

        void begin();

        // Output is filled with outputs from roll and pitch controllers (in that order)
        // refs are roll and pitch reference signals
        void update(float output[2], float refs[2], const State& state);

        void reset();

    private:
        PIDController _roll_controller;
        PIDController _pitch_controller;

};






#endif