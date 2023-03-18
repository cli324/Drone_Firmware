#include "SimpleStateEstimator.h"

SimpleStateEstimator::SimpleStateEstimator(){
    // Setting the initial state
    state.pitch = 0;
    state.roll = 0;
    state.wx = 0;
    state.wy = 0;
    state.wz = 0;
    state.pos_z = 0;
    state.vel_z = 0;

    _accel_x = 0;
    _accel_y = 0;
    _accel_z = -g;

    _acceleration_estimator = SimpleAccelerationEstimator(_accel_x,_accel_y,-g);
    _angular_rate_estimator = SimpleRateEstimator(state.wx, state.wy, state.wz);
    _attitude_estimator = SimpleAttitudeEstimator(state.pitch, state.roll);
    _height_estimator = SimpleHeightEstimator(state.pos_z, state.vel_z);
}

void SimpleStateEstimator::begin(){
    _acceleration_estimator.begin();
    _angular_rate_estimator.begin();
    _attitude_estimator.begin();
    _height_estimator.begin();
}


void SimpleStateEstimator::update(const SensorData& sensor_data){
    if(sensor_data.imu_updated){
        // Updating acceleration, angular rate, and orientation
        float estimated_acceleration[3];
        _acceleration_estimator.update(estimated_acceleration, sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        _accel_x = estimated_acceleration[0];
        _accel_y = estimated_acceleration[1];
        _accel_z = estimated_acceleration[2];

        float estimated_angular_rates[3];
        _angular_rate_estimator.update(estimated_angular_rates, sensor_data.w_x, sensor_data.w_y, sensor_data.w_z);
        state.wx = estimated_angular_rates[0];
        state.wy = estimated_angular_rates[1];
        state.wz = estimated_angular_rates[2];

        float estimated_attitude[2];
        _attitude_estimator.update(estimated_attitude, _accel_x, _accel_y, _accel_z, state.wx, state.wy, state.wz);
        state.pitch = estimated_attitude[0];
        state.roll = estimated_attitude[1];

        // Update vertical position and velocity via IMU dead reckoning
        float estimated_vertical_pos_speed[2];
        _height_estimator.update_dead_reckoning(estimated_vertical_pos_speed, state.pitch, state.roll, _accel_x, _accel_y, _accel_z);
        state.pos_z = estimated_vertical_pos_speed[0];
        state.vel_z = estimated_vertical_pos_speed[1];
    }

    if(sensor_data.barometer_updated){
        // Update vertical position and velocity via barometer measurement
        float estimated_vertical_pos_speed[2];
        _height_estimator.update_measurement(estimated_vertical_pos_speed, sensor_data.pos_z);
        state.pos_z = estimated_vertical_pos_speed[0];
        state.vel_z = estimated_vertical_pos_speed[1];
    }
}
