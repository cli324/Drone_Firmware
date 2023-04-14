#include "BluetoothConnection.h"

BluetoothConnection* BluetoothConnection::connection = nullptr;

BluetoothConnection* BluetoothConnection::get_instance(){
    if(connection == nullptr){
        connection = new BluetoothConnection();
    }
    return connection;
}

void BluetoothConnection::begin(){
    if(!_begun){
        _begun = true;
        _bt.begin(_device_name);
        _heartbeat_send_timer = 0;
        _heartbeat_receive_timer = 0;
    }
}

void BluetoothConnection::update(){
    // Processing incoming bytes
    while(_bt.available()){
        char data = _bt.read();
        // Clear buffer if full
        if(_current_buffer_index > _MAX_MESSAGE_LENGTH){
            _current_buffer_index = 0;
            _start_character_received = false;
        }

        // Checking if character is the start character
        if(data == _START_CHARACTERS[0] && !_start_character_received){
            _read_buffer[0] = data;
            _current_buffer_index = 1;
            _start_character_received = true;
        }
        // Checking if data is an end character
        else if(data == _END_CHARACTER && _current_buffer_index >= 4){ // 2 bytes of start characters, 2 byte checksum = 4
            _read_buffer[_current_buffer_index] = data;
            _current_buffer_index++;
            if(_start_character_received){
                _parse_received_bytes();
            }
            // Clearing buffer
            _current_buffer_index = 0;
            _start_character_received = false;
        }
        else if(_start_character_received){
            _read_buffer[_current_buffer_index] = data;
            _current_buffer_index++;
        }
    }

    // Sending heartbeats
    if(_heartbeat_send_timer > _HEARTBEAT_PERIOD){
        _heartbeat_send_timer = 0;
        _send_heartbeat();
    }

    // Checking if connection has timed out
    if(_heartbeat_receive_timer > _HEARTBEAT_TIMEOUT && _is_connected == true){
        _is_connected = false;
        _system_armed = false;  // Disarm system in event of disconnect
    }
}

void BluetoothConnection::send_battery_percent(uint8_t battery_level){
    uint8_t battery_percent = battery_level;
    if(battery_percent > 100){
        battery_percent = 100;
    }

    if(_is_connected){
        char message_type[2];  // Buffer for converting message type to char. Extra byte for null character.
        sprintf(message_type,"%i",static_cast<uint8_t>(MessageTypes::batteryLevel));

        char message_value[4] = {'\0'};
        sprintf(message_value,"%i",battery_percent);

        char message[4];
        message[0] = message_type[0];
        memcpy(message+1,message_value,sizeof(message) - 1);

        // Only sending bytes corresponding to the message type and the data
        // log10f expression evaluates to the number of digits in battery_percent
        _send_message(message,1 + (uint8_t)(log10f(battery_percent) + 1));
    }
}

void BluetoothConnection::read_joystick_commands(float commands[4]) {
    memcpy(commands,_joystick_commands,sizeof(_joystick_commands));
    _joystick_commands_updated = false;
}

void BluetoothConnection::_send_message(const char message[],const uint8_t message_length){
    if(_is_connected){
        char full_message[message_length+5];
        uint16_t checksum = _checksum(message,0,message_length);
        full_message[0] = _START_CHARACTERS[0];
        full_message[1] = _START_CHARACTERS[1];
        full_message[2] = (char) (checksum & 0b0000000011111111);  // Lower 8 bits
        full_message[3] = (char) (checksum >> 8);  // Upper 8 bits
        memcpy(full_message+4,message,message_length);
        full_message[message_length+4] = _END_CHARACTER;

        for(uint8_t i=0; i<sizeof(full_message)/sizeof(full_message[0]); i++){
            _bt.write(full_message[i]);
        }
    }
}

void BluetoothConnection::_send_heartbeat(){
    if(_is_connected){
        char message[2];
        char message_type[2];  // Buffer for converting message type to char. Extra byte for null character.
        sprintf(message_type,"%i",static_cast<uint8_t>(MessageTypes::heartbeat));

        message[0] = message_type[0];
        message[1] = 0;  // This field is used by the controller to indicate the commanded arm status. Unused when sending heartbeats back to the controller.
        _send_message(message,sizeof(message)/sizeof(message[0]));
    }
}


void BluetoothConnection::_parse_received_bytes(){
    // Checking the message length
    uint8_t start_chars_length = 2; uint8_t end_char_length = 1; uint8_t checksum_length = 2;
    uint8_t min_message_length = start_chars_length + end_char_length + checksum_length + 1;
    if(_current_buffer_index < min_message_length){
        // Invalid message received
        return;
    }

    // Checking the checksum
    uint16_t received_checksum = ((uint16_t)_read_buffer[2]) | (((uint16_t)_read_buffer[3]) << 8);
    uint8_t message_start_index = start_chars_length + checksum_length;
    uint8_t message_end_index = _current_buffer_index - end_char_length;
    uint16_t computed_checksum = _checksum(_read_buffer, message_start_index, message_end_index);

    if(received_checksum != computed_checksum){
        return;
    }

    // Valid message received - processing message contents
    uint8_t non_message_bytes_count = start_chars_length + end_char_length + checksum_length;
    uint8_t message_length = _current_buffer_index - non_message_bytes_count;
    char message[message_length];
    memcpy(message,_read_buffer+4,message_length);

    // Processing message
    char message_type_string[2];
    message_type_string[0] = message[0];
    message_type_string[1] = '\0';

    // Checking that the message type is a valid type
    if(message_type_string[0] < '0' || message_type_string[0] > '9'){
        // Invalid message type
        return;
    }

    uint8_t message_type = (uint8_t) atol(message_type_string);
    switch(message_type){
        case static_cast<uint8_t>(MessageTypes::heartbeat):
            _process_heartbeat_message(message);
            break;
        case static_cast<uint8_t>(MessageTypes::joysticks):
            _process_joysticks_message(message,message_length);
            break;
        case static_cast<uint8_t>(MessageTypes::arm):
            _process_arm_message(message);
            break;
        default:
            break;
    }

}

void BluetoothConnection::_process_heartbeat_message(char heartbeat_message[2]){
    // Unpacking message
    char command_arm_string[2] = {heartbeat_message[1],'\0'};
    bool command_arm = (bool) atol(command_arm_string);

    // Resetting heartbeat receive timer
    _heartbeat_receive_timer = 0;

    _is_connected = true;
    _system_armed = command_arm;
}

void BluetoothConnection::_process_joysticks_message(char joysticks_message[], uint8_t message_length){
    // Copying message
    // Discarding the MessageType byte (the first byte)
    char message[message_length];
    memcpy(message,joysticks_message+1,message_length-1);
    message[message_length-1] = '\0';

    // Unpacking message
    char* endptr = nullptr;
    float commands[4];
    for(int i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        if(endptr >= (message+message_length)-1){return;}  // Invalid message received - should not have reached end of string yet
        if(i==0){
            commands[i] = strtof(message,&endptr);
        }
        else{
            commands[i] = strtof(endptr+1,&endptr);
        }
    }

    memcpy(_joystick_commands,commands,sizeof(commands));
    _joystick_commands_updated = true;
}


void BluetoothConnection::_process_arm_message(char arm_message[2]){
    char arm_message_string[2] = {arm_message[1], '\0'};
    _system_armed = (bool) atol(arm_message_string);
}


uint16_t BluetoothConnection::_checksum(const char byte_array[], uint8_t start_index, uint8_t stop_index){
    uint16_t sum = 0;

    for(uint8_t i=start_index; i<stop_index; i++){
        sum += byte_array[i];
    }

    return sum;
}
