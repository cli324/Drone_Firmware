#ifndef bluetooth_remote_control
#define bluetooth_remote_control

#include <BluetoothSerial.h>
#include "elapsedMillis.h"
#include "Arduino.h"

enum class MessageTypes{
    heartbeat,
    batteryLevel,
    joysticks,
    arm,
};

class BluetoothConnection{
    public:
        // Singleton class - deleting copy constructor and assignment
        BluetoothConnection(const BluetoothConnection& obj) = delete;
        void operator=(const BluetoothConnection&) = delete;
        static BluetoothConnection* get_instance();
        
        void begin();

        void update();

        void send_battery_percent(uint8_t battery_level);

        bool is_connected(){return _is_connected;}

        // commands array is filled with thrust, yaw, pitch, and roll commands (in that order)
        void read_joystick_commands(float commands[4]);
        bool joystick_commands_updated(){return _joystick_commands_updated;}

        // Returns the arm command sent from the mobile application
        // Does not dictate whether or not the system is armed
        // The state machine may choose to disarm the aircraft in the event of a crash or a low battery event
        bool system_armed(){return _system_armed;}

    private:
        // Singleton class
        BluetoothConnection() = default;
        static BluetoothConnection* connection;

        const char _device_name[12] = "ESP32 Drone";
        BluetoothSerial _bt;

        // Flag for whether or not begin method has been called
        bool _begun = false;

        // Connection status to mobile application
        bool _is_connected = false;

        // Buffer for storing bytes received from the bluetooth connection
        static const int _MAX_MESSAGE_LENGTH = 30;
        char _read_buffer[_MAX_MESSAGE_LENGTH] = {};
        uint8_t _current_buffer_index = 0;
        bool _start_character_received = false;

        // Messages exchanged with the phone app are of the following format (without spaces):
        // _START_CHARACTERS CHECKSUM MESSAGE _END_CHARACTER
        const char _START_CHARACTERS[3] = "@ ";
        const char _END_CHARACTER = '\n';

        // Timers for sending and receiving heartbeats
        elapsedMillis _heartbeat_send_timer;
        elapsedMillis _heartbeat_receive_timer;
        const uint16_t _HEARTBEAT_PERIOD = 500;  // Send heartbeats every 500 ms
        const uint16_t _HEARTBEAT_TIMEOUT = 1500;  // Set _is_connected to false if no heartbeat is received after _HEARTBEAT_TIMEOUT ms

        // Storage for processed message values received from the drone
        // Thrust, yaw, pitch, roll
        float _joystick_commands[4] = {0,0,0,0};
        bool _joystick_commands_updated = false;
        bool _system_armed = false;

        //-----------------------------------------Message processing methods-----------------------------------------------
        // Method for writing a c-string message to the bluetooth client
        void _send_message(const char message[],const uint8_t message_length);

        void _send_heartbeat();

        void _parse_received_bytes();

        void _process_heartbeat_message(char heartbeat_message[2]);
        void _process_joysticks_message(char joysticks_message[], uint8_t message_length);
        void _process_arm_message(char arm_message[2]);

        // Simple checksum which consists of adding up the bytes within a message
        // Computes the checksum for the elements in byte_array from start_index (inclusive) to stop_index (exclusive)
        uint16_t _checksum(const char byte_array[], uint8_t start_index, uint8_t stop_index);
};


#endif