#include <SimpleFOC.h>

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// cs              - SPI chip select pin 
// bit_resolution  - sensor resolution
// angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensorSPI as5047u = MagneticSensorSPI(4, 14, 0x3FFF);
// or quick config
// MagneticSensorSPI as5047u = MagneticSensorSPI(10, AS4147_SPI);

int counter;

void setup() {
  // monitoring port
  Serial.begin(9600);
  counter = 0;

  // initialise magnetic sensor hardware
  as5047u.init();

  Serial.println("as5047u ready");
  _delay(1000);
}

void loop() {
  // IMPORTANT - call as frequently as possible
  // update the sensor values 
  as5047u.update();
  // display the angle and the angular velocity to the terminal
  Serial.println(as5047u.getMechanicalAngle()*180/3.14);
  // Serial.print("\t");
  // Serial.println(as5047u.getVelocity());
  _delay(40);
}