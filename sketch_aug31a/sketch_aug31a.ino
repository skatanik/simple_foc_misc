#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6826/MagneticSensorMT6826.h>

MagneticSensorMT6826 sensorMag = MagneticSensorMT6826(4);

void initMySensorCallback(){
  sensorMag.init();
}
// function reading the encoder 
float readMySensorCallback(){
  // return the value in between 0 - 2PI
  return sensorMag.getSensorAngle();
}
// create the generic sensor
GenericSensor sensor = GenericSensor(readMySensorCallback, initMySensorCallback);

int counter;
float current_angle;
float current_velocity;

void setup() {
  // monitoring port
  Serial.begin(9600);
  counter = 0;

  // initialise magnetic sensor hardware
  sensor.init();

  _delay(1000);

  Serial.println("sensor ready");

  _delay(1000);
}

void loop() {
  // IMPORTANT - call as frequently as possible
  // update the sensor values 
  // sensor.update();
  // display the angle and the angular velocity to the terminal
  current_angle = sensor.getAngle();
  Serial.print(current_angle, 4);
  Serial.print("   ");
  sensor.update();
  current_velocity = sensor.getVelocity();
  Serial.println(current_velocity, 4);
  // Serial.print("\t");
  // Serial.println(as5047u.getVelocity());
  _delay(50);
}