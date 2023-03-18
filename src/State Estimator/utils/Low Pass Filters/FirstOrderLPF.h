#ifndef FIRST_ORDER_LPF
#define FIRST_ORDER_LPF

#include "elapsedMillis.h"

// First order low pass filter
class FirstOrderLPF{
    public:
        FirstOrderLPF(float cutoff_frequency_hz);
        FirstOrderLPF(float cutoff_frequency_hz,float initial_value);

        void begin();

        float filter(float value);


    private:
        float _w0;  // Cutoff frequency in rad/s
        float _prev;  // Previous output from the filter
        bool _initial_value_received = false;
        elapsedMicros _timer;  // Timer to measure time between calls to filter
};





#endif