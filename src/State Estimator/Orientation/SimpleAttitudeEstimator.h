#ifndef SIMPLE_ATTITUDE_ESTIMATOR
#define SIMPLE_ATTITUDE_ESTIMATOR

#include "elapsedMillis.h"
#include "../../constants.h"

// Roll and pitch estimation via a complementary filter
// This estimator is only valid for pitch within the domain [-pi/2, pi/2]
class SimpleAttitudeEstimator{
    public:
        SimpleAttitudeEstimator(float pitch, float roll);
        SimpleAttitudeEstimator() = default;

        void begin();

        void update(float attitude[2], float accel_x, float accel_y, float accel_z, float w_x, float w_y, float w_z);

    private:
        elapsedMicros _timer;  // Tracks time between calls to "update"
        float _pitch = 0;
        float _roll = 0;
        float _alpha = 0.001;  // Coefficient in complementary filter

        // Pitch and roll are constrained to (-pi,pi]
        // Helper functions for adding angles
        float _constrain_to_pi(float angle);  // Constrains angle to (-pi,pi]
        float _constrain_to_2_pi(float angle);  // Constrains angle to [0,2*pi)
        float _weighted_average_of_angles(float angle_1, float weight_1, float angle_2, float weight_2);
};





#endif