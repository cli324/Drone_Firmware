#include "SimpleRateEstimator.h"

SimpleRateEstimator::SimpleRateEstimator(float wx, float wy, float wz){
    _wx_filter = FirstOrderLPF(_cutoff_frequency, wx);
    _wy_filter = FirstOrderLPF(_cutoff_frequency, wy);
    _wz_filter = FirstOrderLPF(_cutoff_frequency, wz);
}

void SimpleRateEstimator::begin(){
    _wx_filter.begin();
    _wy_filter.begin();
    _wz_filter.begin();
}

void SimpleRateEstimator::update(float estimated_rates[3],float wx_raw, float wy_raw, float wz_raw){
    float wx = _wx_filter.filter(wx_raw - _biases.w_x_bias);
    float wy = _wy_filter.filter(wy_raw - _biases.w_y_bias);
    float wz = _wz_filter.filter(wz_raw - _biases.w_z_bias);

    estimated_rates[0] = wx;
    estimated_rates[1] = wy;
    estimated_rates[2] = wz;
}