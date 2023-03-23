#ifndef SIMPLE_HEIGHT_ESTIMATOR
#define SIMPLE_HEIGHT_ESTIMATOR

#include <BasicLinearAlgebra.h>
#include "../../constants.h"
#include "elapsedMillis.h"

// Estimates vertical position and vertical velocity using a Kalman filter
class SimpleHeightEstimator{
    public:
        SimpleHeightEstimator(float initial_height, float initial_vertical_speed);
        SimpleHeightEstimator() = default;

        void begin();

        // Estimated state returned in the estimated_state array
        // State is defined as [vertical_position, vertical_speed]. Units are m and m/s
        // First update function performs dead-reckoning with IMU data only
        // Second update function performs a measurement update step with barometer data only
        void update_dead_reckoning(float estimated_state[2], float pitch, float roll, float acc_x, float acc_y, float acc_z);
        void update_measurement(float estimated_state[2], float pos_z);
        
    private:
        float _imu_to_vertical_acceleration(float pitch, float roll, float acc_x, float acc_y, float acc_z);

        // Kalman filter prediction step
        void _kalman_filter_prediction(BLA::Matrix<2>& mu_bar, BLA::Matrix<2,2>& sigma_bar, float vertical_acceleration, float deltaT);

        BLA::Matrix<2,2> R = {0.01,0,0,0.01};  // Process noise covariance
        BLA::Matrix<1,1> Q = {5.0f};  // Measurement noise covariance
        BLA::Matrix<1,2> C = {1,0};  // Measurement update matrix
        BLA::Matrix<2> mu_prev = {0,0};  // Previous state prediction. State is defined as [vertical_position, vertical_speed]
        BLA::Matrix<2,2> sigma_prev = {3,0,0,3};  // Initialize covariance matrix with high level of uncertainty

        elapsedMicros _timer;
};







#endif