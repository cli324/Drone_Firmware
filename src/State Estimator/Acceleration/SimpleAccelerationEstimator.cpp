#include "SimpleAccelerationEstimator.h"

SimpleAccelerationEstimator::SimpleAccelerationEstimator(float accel_x, float accel_y, float accel_z):
    _accel_x(accel_x), _accel_y(accel_y), _accel_z(accel_z){
        _accel_x_filter = FirstOrderLPF(_cutoff_frequency,_accel_x);
        _accel_y_filter = FirstOrderLPF(_cutoff_frequency,_accel_y);
        _accel_z_filter = FirstOrderLPF(_cutoff_frequency,_accel_z);
    }

void SimpleAccelerationEstimator::begin(){
    _accel_x_filter.begin();
    _accel_y_filter.begin();
    _accel_z_filter.begin();
}

void SimpleAccelerationEstimator::update(float estimated_acceleration[3],float accel_x, float accel_y, float accel_z){
    _accel_x = _accel_x_filter.filter(accel_x - _biases.acc_x_bias);
    _accel_y = _accel_y_filter.filter(accel_y - _biases.acc_y_bias);
    _accel_z = _accel_z_filter.filter(accel_z - _biases.acc_z_bias);

    estimated_acceleration[0] = _accel_x;
    estimated_acceleration[1] = _accel_y;
    estimated_acceleration[2] = _accel_z;
}