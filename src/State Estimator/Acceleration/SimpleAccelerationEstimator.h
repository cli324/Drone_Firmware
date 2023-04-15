#ifndef SIMPLE_ACCELERATION_ESTIMATOR
#define SIMPLE_ACCELERATION_ESTIMATOR

#include "../IMU Biases/FixedIMUBiases.h"
#include "../../utils/Low Pass Filters/FirstOrderLPF.h"

// Subtracts IMU biases and applies low pass filtering to raw accelerometer data
class SimpleAccelerationEstimator{
    public:
        SimpleAccelerationEstimator(float accel_x, float accel_y, float accel_z);
        SimpleAccelerationEstimator() = default;

        void begin();

        void update(float estimated_acceleration[3],float accel_x, float accel_y, float accel_z);
    
    private:
        float _cutoff_frequency = 30;
        FirstOrderLPF _accel_x_filter = FirstOrderLPF(_cutoff_frequency);  // 30 hz cutoff frequency low pass filter
        FirstOrderLPF _accel_y_filter = FirstOrderLPF(_cutoff_frequency);
        FirstOrderLPF _accel_z_filter = FirstOrderLPF(_cutoff_frequency);

        float _accel_x;
        float _accel_y;
        float _accel_z;

        static constexpr IMUBiases _biases = IMUBiases();
};





#endif