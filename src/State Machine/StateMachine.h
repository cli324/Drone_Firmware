#ifndef ESP32_DRONE_STATE_MACHINE
#define ESP32_DRONE_STATE_MACHINE

#include "../Bluetooth/BluetoothConnection.h"
#include "../Sensors/SensorData.h"
#include "../State Estimator/State.h"
#include "elapsedMillis.h"

/*
State machine functions:
- Control system arm/disarm state
- Detect crashes
- Monitor battery voltage
- Check bluetooth connection status
*/

class StateMachine{
    public:
        StateMachine() = default;

        void begin();

        void update(const SensorData& sensor_data, const State& state);

        bool system_armed(){return _armed;}

    private:
        // BluetoothConnection is a singleton class
        BluetoothConnection* _bluetooth = BluetoothConnection::get_instance();

        // State variables
        bool _armed = false;
        bool _crashed = false;
        bool _low_battery = false;
        bool _out_of_battery = false;
        bool _bluetooth_connected = false;

        // Constants for calculating % battery remaining
        const float _MAX_BATTERY_VOLTAGE = 4.35;  // 1s LiHV battery
        const float _MIN_BATTERY_VOLTAGE = 3.5;
        uint8_t _battery_percent_remaining = 0;
        const uint8_t _LOW_BATTERY_LEVEL = 10;
        const uint8_t _OUT_OF_BATTERY_LEVEL = 0;

        // Constants for determining whether or not a crash has occurred
        // Assuming that the system has crashed if pitch or roll angle is greater than a threshold
        const float _MAX_TILT_RAD = 60.0 * PI / 180.0;

        // Timers
        elapsedMillis _update_timer;
        elapsedMillis _battery_message_timer;
        const uint16_t _UPDATE_PERIOD_MS = 5;
        const uint16_t _BATTERY_MESSAGE_SEND_PERIOD_MS = 500;
};






#endif