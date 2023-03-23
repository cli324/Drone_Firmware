#ifndef SIMPLE_STATE_ESTIMATOR
#define SIMPLE_STATE_ESTIMATOR

#include "Arduino.h"
#include "../constants.h"
#include "../Sensors/SensorData.h"
#include "Acceleration/SimpleAccelerationEstimator.h"
#include "Angular Rate/SimpleRateEstimator.h"
#include "Orientation/SimpleAttitudeEstimator.h"
#include "Vertical Position/SimpleHeightEstimator.h"

struct State{
    float pitch;
    float roll;
    float wx;
    float wy;
    float wz;
    float pos_z;
    float vel_z;

    float accel_x;
    float accel_y;
    float accel_z;
};

class SimpleStateEstimator{
    public:
        SimpleStateEstimator();

        void begin();

        // Updates the system state
        void update(const SensorData& sensor_data);

        State state;

    private:
        SimpleAccelerationEstimator _acceleration_estimator;
        SimpleRateEstimator _angular_rate_estimator;
        SimpleAttitudeEstimator _attitude_estimator;
        SimpleHeightEstimator _height_estimator;
};







#endif