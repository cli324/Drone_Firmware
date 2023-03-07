#include "src/Sensors/Sensor_Hub.h"
#include "src/Motors/Motor_Hub.h"
#include "elapsedMillis.h"

SensorHub sensors;
MotorHub motors;

elapsedMillis system_timer;


void setup() {
  Serial.begin(115200);
  while(!Serial){}

  motors.begin();
  sensors.begin();
  system_timer = 0;
}

void loop() {
  sensors.update();
  if(sensors.data.barometer_updated){
    Serial.println(sensors.data.pos_z);
    sensors.data.barometer_updated = false;
  }
}
