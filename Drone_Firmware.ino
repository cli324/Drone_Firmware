#include "elapsedMillis.h"

#include "src/Sensors/Sensor_Hub.h"
#include "src/Motors/Motor_Hub.h"
#include "src/State Estimator/SimpleStateEstimator.h"

SensorHub sensors;
MotorHub motors;
SimpleStateEstimator state_estimator;

elapsedMillis system_timer;


void setup() {
  Serial.begin(115200);
  while(!Serial){} 

  motors.begin();
  sensors.begin();
  state_estimator.begin();
  system_timer = 0;
}

void loop() {
  sensors.update();
  if(sensors.data.barometer_updated || sensors.data.imu_updated){
    state_estimator.update(sensors.data);

    sensors.data.barometer_updated = false;
    sensors.data.imu_updated = false;
  }

  if(system_timer > 20){
    system_timer -= 20;
    Serial.println(state_estimator.state.pos_z);
  }
}
