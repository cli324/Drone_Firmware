#ifndef ESP32_DRONE_RATE_CONTROLLER
#define ESP32_DRONE_RATE_CONTROLLER

#include "../utils/PID Controller/PIDController.h"
#include "../../State Estimator/State.h"

class RateController{
    public:
        RateController(float pitch_roll_kp, float pitch_roll_ki, float pitch_roll_kd, float yaw_kp, float yaw_ki, float yaw_kd);

        void begin();

        // Outputs is populated with outputs from roll, pitch, and yaw rate controllers (in that order)
        // refs are roll, pitch, and yaw rate reference signals
        void update(float outputs[3], float refs[3], const State& state);

        void reset();

    private:
        const float _derivative_cutoff_frequency = 30;
        PIDController _pitch_rate_controller;
        PIDController _roll_rate_controller;
        PIDController _yaw_rate_controller;
};










#endif