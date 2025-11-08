#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6826/MagneticSensorMT6826.h>
#include <math.h>

#define   EN_GATE PB4
#define   M_PWM PB6
#define   M_OC PC11
#define   OC_ADJ PA15
#define   IOUTA A0
#define   IOUTB A1
#define   IOUTC A2
#define   OC_GAIN PB5

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

// motor instance
BLDCMotor motor = BLDCMotor(11, 0.193, 380);

// driver instance
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15, EN_GATE);

// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
LowsideCurrentSense current_sense =  LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

float target_Int = 0;

float max_velocity;

Commander command = Commander(Serial);
void doTarget(char* cmd) { 
  float target = atof(cmd);
  float target_fin;
  float max_velocity_new;
  target_fin = motor.shaft_angle + 0.2*target + 0.0*target_Int;
  target_Int+=target;

  max_velocity_new = max_velocity*abs(target_fin - motor.shaft_angle) / 3.14;

  motor.velocity_limit = max_velocity_new;

  if(target_fin > 3.14/2.0)
  {
    target_fin = 3.14/2.0;
  } else if(target_fin < -3.14/2.0) {
    target_fin = -3.14/2.0;
  }
  
  motor.target = target_fin;
  Serial.println(target_fin);
  // Serial.println(max_velocity_new);
  // command.scalar(&motor.target, cmd); 
  
  }

void sendAngle(char* cmd) { 
  command.scalar(&motor.target, cmd); 
  }

long time_ms = 0;

  // For example, we will create a timer that runs at 10kHz on the TIM5
  HardwareTimer* timer = new HardwareTimer(TIM3);

void setup() {

  max_velocity = 80;

  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 6pwm mode (can be left open)
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,LOW);
  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);

  sensor.init();

  motor.linkSensor(&sensor);

// driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 24000;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // link the driver to the current sense
  current_sense.linkDriver(&driver);
 
  // align voltage
  motor.voltage_sensor_align = 2.0;
  motor.zero_electric_angle = 1.18;
  motor.sensor_direction = Direction::CCW;

  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

// control loop type and torque mode 
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;
  // motor.motion_downsample = 1.0;

  // velocity loop PID
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 0.01;
  motor.LPF_velocity.Tf = 0.007;

  motor.PID_velocity.output_ramp = 300;
  
  // angle loop PID
  // motor.P_angle.P = 100.0;
  motor.P_angle.P = 50.0;
  motor.LPF_angle.Tf = 0.005;

   // foc current control parameters (Arduino UNO/Mega)
  motor.PID_current_q.P = 2;
  motor.PID_current_q.I= 10;
  motor.PID_current_d.P= 2;
  motor.PID_current_d.I = 10;

  motor.LPF_current_q.Tf = 0.005;
  motor.LPF_current_d.Tf = 0.005;
  
    // Limits 
  motor.velocity_limit = max_velocity; 
  motor.voltage_limit = 23.0;   // 12 Volt limit 
  motor.current_limit = 8.0;    // 2 Amp current limit


  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);



  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  motor.sensor_offset = motor.shaft_angle;
  // motor.target = motor.shaft_angle;

  // motor.target = 4.5;

  // add target command T
  command.add('T', doTarget, "target position");
  command.add('A', sendAngle, "shaft angle");
  

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));

  ///////////////////////

  
  // // create a hardware timer

  // // Set timer frequency to 10kHz
  // timer->setOverflow(1000, HERTZ_FORMAT); 
  // // add the loopFOC and move to the timer
  // timer->attachInterrupt([](){
  //   // call the loopFOC and move functions
  //   motor.loopFOC();
  //   motor.move();
  // });
  // // start the timer
  // timer->resume();

  ////////////////////////

  _delay(1000);

    time_ms = millis();

}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();

        // if (millis() - time_ms > 100){
        //   Serial.print(motor.target);
        //   Serial.print("\t");
        //   Serial.println("");
        //   time_ms = millis();
        // }
    // PhaseCurrent_s current = current_sense.getPhaseCurrents();
    // Serial.print(current.a);
    // Serial.print("\t");
    // Serial.print(current.b);
    // Serial.print("\t");
    // Serial.print(current.c);
    // Serial.println("");
    // time_ms = millis();
    // }
}
