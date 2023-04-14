#ifndef SIMPLE_STATE_ESTIMATOR
#define SIMPLE_STATE_ESTIMATOR

#include "Arduino.h"
#include "../constants.h"
#include "../Sensors/SensorData.h"
#include "Acceleration/SimpleAccelerationEstimator.h"
#include "Angular Rate/SimpleRateEstimator.h"
#include "Orientation/SimpleAttitudeEstimator.h"
#include "Vertical Position/SimpleHeightEstimator.h"

#include "State.h"

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