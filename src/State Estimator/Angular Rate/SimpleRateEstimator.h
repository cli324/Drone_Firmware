#ifndef SIMPLE_RATE_ESTIMATOR
#define SIMPLE_RATE_ESTIMATOR

#include "../IMU Biases/FixedImuBiases.h"
#include "../../utils/Low Pass Filters/FirstOrderLPF.h"

// Applies bias subtraction + low pass filtering to raw gyro data
class SimpleRateEstimator{
    public:
        SimpleRateEstimator(float wx, float wy, float wz);
        SimpleRateEstimator() = default;

        void begin();

        // Fills estimated_rates with wx, wy, wz filtered via first order LPF
        void update(float estimated_rates[3],float wx_raw, float wy_raw, float wz_raw);

    private:
        static constexpr IMUBiases _biases = IMUBiases();
        float _cutoff_frequency = 30;  // 30 hz cutoff frequency
        FirstOrderLPF _wx_filter = FirstOrderLPF(_cutoff_frequency);
        FirstOrderLPF _wy_filter = FirstOrderLPF(_cutoff_frequency);
        FirstOrderLPF _wz_filter = FirstOrderLPF(_cutoff_frequency);        
};




#endif