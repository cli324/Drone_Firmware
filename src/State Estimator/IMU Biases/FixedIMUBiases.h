#ifndef FIXED_IMU_BIASES
#define FIXED_IMU_BIASES

#include "../../constants.h"

// May develop an IMU bias estimator in the future
struct IMUBiases{
    static constexpr float acc_x_bias = -0.04759596250901226;
    static constexpr float acc_y_bias = 0.23f;
    static constexpr float acc_z_bias = -9.915927090843548 + g;
    static constexpr float w_x_bias = -0.0004935472242249459;
    static constexpr float w_y_bias = -0.001220241528478731;
    static constexpr float w_z_bias = -0.004148323720259553;
};




#endif