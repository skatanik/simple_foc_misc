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
#define OC_GAIN PB5


// motor instance
BLDCMotor motor = BLDCMotor(22, 0.1932, 380, 0.0000277221915894188);

// driver instance
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15, EN_GATE);

// MXLEMMING observer sensor instance
MXLEMMINGObserverSensor observer = MXLEMMINGObserverSensor(motor);

// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
LowsideCurrentSense current_sense =  LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

// commander communication instance
Commander command = Commander(Serial);
void doMotor(char* cmd){ command.motor(&motor, cmd); }

long time_ms = 0;

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // link the motor to the sensor
  motor.linkSensor(&observer);

    // DRV8302 specific code
  // M_OC  - enable over-current protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 6pwm mode (can be left open)
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,LOW);
  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);

  pinMode(OC_GAIN,OUTPUT);
  digitalWrite(OC_GAIN,LOW);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 20000; // suggested under 18khz
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  current_sense.linkDriver(&driver);

    // align voltage
  motor.voltage_sensor_align = 2;

  // set control loop type to be used
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;

  // current sense init and linking
  current_sense.init();
  current_sense.skip_align = true;
  // driver 8302 has inverted gains on all channels
  current_sense.gain_a *= -1;
  current_sense.gain_b *= -1;
  current_sense.gain_c *= -1;
  motor.linkCurrentSense(&current_sense);

  // initialise motor
  motor.init();

motor.target= 0.0;

  // skip the sensor alignment
  motor.sensor_direction= Direction::CW;
  motor.zero_electric_angle = 0;
  motor.initFOC();


  // subscribe motor to the commander
  command.add('M', doMotor, "motor");
  
  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println("Motor ready.");

  _delay(1000);

  time_ms = millis();
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move();

  // motor monitoring
  motor.monitor();

  // user communication
  command.run();

    if (millis() - time_ms > 100){
    PhaseCurrent_s current = current_sense.getPhaseCurrents();
    Serial.print(current.a);
    Serial.print("\t");
    Serial.print(current.b);
    Serial.print("\t");
    Serial.print(current.c);
    Serial.println("");
    time_ms = millis();
    }
}