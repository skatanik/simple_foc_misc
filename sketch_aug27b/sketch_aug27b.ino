#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"

// DRV8302 pins connections
// don't forget to connect the common ground pin

#define   EN_GATE PB4
#define   M_PWM PB6 
#define   M_OC PC11
#define   OC_ADJ PA15
#define   IOUTA A0
#define   IOUTB A1
#define   IOUTC A2
#define   OC_GAIN PB5


// motor instance
BLDCMotor motor = BLDCMotor(22);

// driver instance
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15, EN_GATE);


// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
LowsideCurrentSense current_sense =  LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);


void setup() {
// use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

    // DRV8302 specific code
  // M_OC  - enable overcurrent protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  // OD_ADJ - set the maximum overcurrent limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);


  // pinMode(OC_GAIN,OUTPUT);
  // digitalWrite(OC_GAIN,LOW);

  //   pinMode(EN_GATE,OUTPUT);
  // digitalWrite(EN_GATE,HIGH);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  current_sense.linkDriver(&driver);

  // current sense init and linking
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // initialise motor
  motor.init();

  // find the motor parameters
  motor.characteriseMotor(5.5f);


  _delay(1000);

  pinMode(EN_GATE,OUTPUT);
  digitalWrite(EN_GATE,LOW);
}


void loop() {
  _delay(1000);
}