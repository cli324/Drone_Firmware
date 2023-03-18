#include "SimpleAttitudeEstimator.h"

SimpleAttitudeEstimator::SimpleAttitudeEstimator(float pitch, float roll){
    _pitch = pitch;
    _roll = roll;
}

void SimpleAttitudeEstimator::begin(){
    _timer = 0;
}

void SimpleAttitudeEstimator::update(float attitude[2], float accel_x, float accel_y, float accel_z, float w_x, float w_y, float w_z){
    float deltaT = ((float) _timer) / 1000000.0f;
    _timer = 0;  // Reset timer

    // Setting acceleration vector to magnitude g
    float acceleration_norm = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    bool use_acceleration_data = true;
    if(acceleration_norm == 0){
        // Cannot use acceleration data
        use_acceleration_data = false;
    }

    // Estimating attitude via the accelerometer
    float accelerometer_roll_estimate;
    float accelerometer_pitch_estimate;
    if(use_acceleration_data){

        float accel_x_normalized = g * accel_x / acceleration_norm;
        float accel_y_normalized = g * accel_y / acceleration_norm;
        float accel_z_normalized = g * accel_z / acceleration_norm;

        accelerometer_roll_estimate = atan2f(-accel_x_normalized,accel_z_normalized);

        accelerometer_pitch_estimate = asinf(accel_y_normalized/g);
    }

    // Estimating attitude via the gyro
    float pitch_dot = w_x * cosf(_roll) + w_z * sinf(_roll);
    float roll_dot = w_y;
    float gyro_pitch_estimate = _constrain_to_pi(_pitch + pitch_dot * deltaT);
    float gyro_roll_estimate = _constrain_to_pi(_roll + roll_dot * deltaT);

    if(use_acceleration_data){
        _pitch = _weighted_average_of_angles(accelerometer_pitch_estimate, _alpha, gyro_pitch_estimate, 1.0f - _alpha);
        _roll = _weighted_average_of_angles(accelerometer_roll_estimate, _alpha, gyro_roll_estimate, 1.0f - _alpha);
    }
    else{
        _pitch = gyro_pitch_estimate;
        _roll = gyro_roll_estimate;
    }
    

    attitude[0] = _pitch;
    attitude[1] = _roll;
}

float SimpleAttitudeEstimator::_constrain_to_pi(float angle){
    float angle_2pi = fmodf(angle,2*PI);
    if(angle_2pi > PI){
        return angle_2pi - 2*PI;
    }
    else if(angle_2pi <= -PI){
        return angle_2pi + 2*PI;
    }
    else{
        return angle_2pi;
    }
}

float SimpleAttitudeEstimator::_constrain_to_2_pi(float angle){
    float angle_2pi = fmodf(angle,2*PI);
    if(angle_2pi < 0){
        return angle_2pi + 2*PI;
    }
    else{
        return angle_2pi;
    }
}


float SimpleAttitudeEstimator::_weighted_average_of_angles(float angle_1, float weight_1, float angle_2, float weight_2){
    float constrained_1 = _constrain_to_pi(angle_1);
    float constrained_2 = _constrain_to_pi(angle_2);
    float diff = constrained_1 - constrained_2;

    if(fabsf(diff) <= PI){
        return constrained_1 * weight_1 + constrained_2 * weight_2;
    }
    else{
        constrained_1 = _constrain_to_2_pi(constrained_1);
        constrained_2 = _constrain_to_2_pi(constrained_2);
        float average = constrained_1 * weight_1 + constrained_2 * weight_2;
        return _constrain_to_pi(average);
    }
}



