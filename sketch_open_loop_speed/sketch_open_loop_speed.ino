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
BLDCMotor motor = BLDCMotor(11, 0.193, 380);

// driver instance
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15, EN_GATE);


// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
LowsideCurrentSense current_sense =  LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimitCurrent(char* cmd) { command.scalar(&motor.current_limit, cmd); }
long time_ms = 0;

void setup() {

    pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 6pwm mode (can be left open)
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,LOW);
  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);
// driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 20000;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  current_sense.linkDriver(&driver);


  // limiting motor current (provided resistance)
   motor.voltage_limit = 20;   // Volts
 
    // align voltage
  motor.voltage_sensor_align = 2;

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // current sense init and linking
  current_sense.init();
  current_sense.skip_align = true;
  // driver 8302 has inverted gains on all channels
  current_sense.gain_a *= -1;
  current_sense.gain_b *= -1;
  current_sense.gain_c *= -1;

  // init motor hardware
  motor.init();
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('C', doLimitCurrent, "current limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);

    time_ms = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.loopFOC();
  // open loop velocity movement
  // using motor.current_limit and motor.velocity_limit
  motor.move();

  // user communication
  command.run();

      if (millis() - time_ms > 20){
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
