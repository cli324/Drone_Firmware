#ifndef ESP32_DRONE_PID_CONTROLLER
#define ESP32_DRONE_PID_CONTROLLER

#include "elapsedMillis.h"
#include "../../../utils/Low Pass Filters/FirstOrderLPF.h"

/*
PID control class with option to apply low pass filtering to the derivative term.
Derivative term is placed on the feedback path to avoid derivative kick.
*/
class PIDController{
    public:
        PIDController(float kp, float ki, float kd, float derivative_lpf_cutoff = 0);

        // Returns the controller output
        float update(float ref, float feedback);

        void begin();

        void reset();

    private:
        const float _kp;
        const float _ki;
        const float _kd;

        float _error_integral = 0;

        FirstOrderLPF _derivative_lpf;
        bool _filter_derivative = false;

        float _signal_prev;  // Value of previous feedback signal passed via update
        bool _first_signal_received = false;  // Flag to indicate whether _signal_prev has been initialized

        elapsedMicros _timer;

        // Calculate proportional, integral, and derivative terms
        float _proportional(float ref, float feedback);
        float _integral(float ref, float feedback, float delta_t_seconds);
        float _derivative(float feedback, float delta_t_seconds);
};





#endif