#include "elapsedMillis.h"

#include "src/Sensors/Sensor_Hub.h"
#include "src/Motors/Motor_Hub.h"
#include "src/State Estimator/SimpleStateEstimator.h"
#include "src/Bluetooth/BluetoothConnection.h"
#include "src/State Machine/StateMachine.h"

#include "src/Control/Controls.h"

SensorHub sensors;
MotorHub motors;
SimpleStateEstimator state_estimator;
BluetoothConnection* bluetooth = BluetoothConnection::get_instance();
StateMachine state_machine;
Controls controls;

elapsedMicros system_timer;


void setup() {
  Serial.begin(115200);
  while(!Serial){} 

  motors.begin();
  sensors.begin();
  state_estimator.begin();
  bluetooth->begin();
  state_machine.begin();
  controls.begin();
  system_timer = 0;

  // Testing
  ledcSetup(4,4000,10);
  ledcAttachPin(2,4);
  ledcWrite(4,0);
}

void loop() {
  sensors.update();
  
  if(sensors.data.barometer_updated || sensors.data.imu_updated){
    state_estimator.update(sensors.data);

    sensors.data.barometer_updated = false;
    sensors.data.imu_updated = false;
  }
  
  bluetooth->update();

  state_machine.update(sensors.data, state_estimator.state);

  if(bluetooth->joystick_commands_updated()){
    float commands[4];
    bluetooth->read_joystick_commands(commands);
    controls.update_joystick_signals(state_machine.system_armed(),commands);
  }
  controls.update(state_machine.system_armed(),state_estimator.state);

  if(state_machine.system_armed()){
    if(controls.new_motor_setpoint_available()){
      float motor_setpoints[4];
      controls.get_motor_setpoints(motor_setpoints);
      /*Serial.print("Front left: ");
      Serial.print(motor_setpoints[0],3);
      Serial.print(" Front right: ");
      Serial.print(motor_setpoints[1], 3);
      Serial.print(" Back right: ");
      Serial.print(motor_setpoints[2], 3);
      Serial.print(" Back left: ");
      Serial.println(motor_setpoints[3], 3);*/
      motors.set(motor_setpoints[0],motor_setpoints[1],motor_setpoints[2],motor_setpoints[3]);
    }
  }
  else{
    // Turn system off
    motors.set(0,0,0,0);
  }

}
