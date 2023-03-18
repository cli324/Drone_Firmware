#include "SimpleHeightEstimator.h"

SimpleHeightEstimator::SimpleHeightEstimator(float initial_height, float initial_vertical_speed){
    mu_prev(0) = initial_height;
    mu_prev(1) = initial_vertical_speed;
}

void SimpleHeightEstimator::begin(){
    _timer = 0;
}

void SimpleHeightEstimator::update_dead_reckoning(float estimated_state[2], float pitch, float roll, float acc_x, float acc_y, float acc_z){
    float vertical_acceleration = _imu_to_vertical_acceleration(pitch,roll,acc_x,acc_y,acc_z);

    // Only performing prediction step of Kalman filter update
    float deltaT = ((float)_timer) / 1000000.0f;
    _timer = 0;
    BLA::Matrix<2> mu_bar; BLA::Matrix<2,2> sigma_bar;
    _kalman_filter_prediction(mu_bar,sigma_bar,vertical_acceleration,deltaT);

    estimated_state[0] = mu_bar(0);
    estimated_state[1] = mu_bar(1);

    mu_prev = mu_bar;
    sigma_prev = sigma_bar;
}

void SimpleHeightEstimator::update_measurement(float estimated_state[2], float pos_z){
    BLA::Matrix<2> mu_bar = mu_prev;
    BLA::Matrix<2,2> sigma_bar = sigma_prev;

    // Kalman filter measurement update step
    BLA::Matrix<1> inverted_term = C*sigma_bar*(~C) + Q;
    BLA::Invert(inverted_term);
    BLA::Matrix<2> K = (sigma_bar * (~C)) * inverted_term;
    BLA::Matrix<2> mu = mu_bar + K*(pos_z - (C*mu_bar)(0));
    BLA::Matrix<2,2> sigma = (BLA::Identity<2>() - K*C)*sigma_bar;

    estimated_state[0] = mu(0);
    estimated_state[1] = mu(1);

    mu_prev = mu;
    sigma_prev = sigma;
}

void SimpleHeightEstimator::_kalman_filter_prediction(BLA::Matrix<2>& mu_bar, BLA::Matrix<2,2>& sigma_bar, float vertical_acceleration, float deltaT){
    BLA::Matrix<2,2> A = {1,deltaT,0,1};
    BLA::Matrix<2> B = {0,deltaT};
    mu_bar = A*mu_prev + B*vertical_acceleration;
    sigma_bar = (A*sigma_prev*(~A)) + R;
}

float SimpleHeightEstimator::_imu_to_vertical_acceleration(float pitch, float roll, float acc_x, float acc_y, float acc_z){
    return -acc_x*cosf(pitch)*sinf(roll) + acc_y*sinf(pitch) + acc_z*cosf(pitch)*cosf(roll) - g;
}

