#ifndef ESP32_DRONE_STATE
#define ESP32_DRONE_STATE

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


#endif