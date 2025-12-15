#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/mt6826/MagneticSensorMT6826.h>
#include <ACANFD_STM32.h>


#define   EN_GATE PB4
#define   M_PWM PB9 //b9
#define   M_OC PC11
#define   OC_ADJ PA15
#define   IOUTA A0
#define   IOUTB A1
#define   IOUTC A2
#define   OC_GAIN PB7 //b7

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

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

long time_ms = 0;

  // For example, we will create a timer that runs at 10kHz on the TIM5
  HardwareTimer* timer = new HardwareTimer(TIM3);

void setup() {

  _delay(1000);

  Serial.print ("CPU frequency: ") ;
  Serial.print (F_CPU) ;
  Serial.println (" Hz") ;
  Serial.print ("PCLK1 frequency: ") ;
  Serial.print (HAL_RCC_GetPCLK1Freq ()) ;
  Serial.println (" Hz") ;
  Serial.print ("PCLK2 frequency: ") ;
  Serial.print (HAL_RCC_GetPCLK2Freq ()) ;
  Serial.println (" Hz") ;
  Serial.print ("HCLK frequency: ") ;
  Serial.print (HAL_RCC_GetHCLKFreq ()) ;
  Serial.println (" Hz") ;
  Serial.print ("SysClock frequency: ") ;
  Serial.print (HAL_RCC_GetSysClockFreq ()) ;
  Serial.println (" Hz") ;
  Serial.print ("FDCAN Clock: ") ;
  Serial.print (fdcanClock ()) ;
  Serial.println (" Hz") ;

  ACANFD_STM32_Settings settings (500 * 1000, DataBitRateFactor::x2) ;
  settings.mModuleMode = ACANFD_STM32_Settings::INTERNAL_LOOP_BACK ;

  Serial.print ("Bit Rate prescaler: ") ;
  Serial.println (settings.mBitRatePrescaler) ;
  Serial.print ("Arbitration Phase segment 1: ") ;
  Serial.println (settings.mArbitrationPhaseSegment1) ;
  Serial.print ("Arbitration Phase segment 2: ") ;
  Serial.println (settings.mArbitrationPhaseSegment2) ;
  Serial.print ("Arbitration SJW: ") ;
  Serial.println (settings.mArbitrationSJW) ;
  Serial.print ("Actual Arbitration Bit Rate: ") ;
  Serial.print (settings.actualArbitrationBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Arbitration sample point: ") ;
  Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
  Serial.println ("%") ;
  Serial.print ("Exact Arbitration Bit Rate ? ") ;
  Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
  Serial.print ("Data Phase segment 1: ") ;
  Serial.println (settings.mDataPhaseSegment1) ;
  Serial.print ("Data Phase segment 2: ") ;
  Serial.println (settings.mDataPhaseSegment2) ;
  Serial.print ("Data SJW: ") ;
  Serial.println (settings.mDataSJW) ;
  Serial.print ("Actual Data Bit Rate: ") ;
  Serial.print (settings.actualDataBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Data sample point: ") ;
  Serial.print (settings.dataSamplePointFromBitStart ()) ;
  Serial.println ("%") ;
  Serial.print ("Exact Data Bit Rate ? ") ;
  Serial.println (settings.exactDataBitRate () ? "yes" : "no") ;

//--- beginFD is called without any receive filter, all sent frames are received
//    by receiveFD0 throught receiveFIFO0
  uint32_t errorCode = fdcan1.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan1 configuration ok") ;
  }else{
    Serial.print ("Error fdcan1: 0x") ;
    Serial.println (errorCode, HEX) ;
  }




  pinMode(PC6,OUTPUT);
  digitalWrite(PC6, HIGH);

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
  // motor.zero_electric_angle = 1.18;
  // motor.sensor_direction = Direction::CCW;

  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

// control loop type and torque mode 
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;
  // motor.motion_downsample = 1.0;

  // velocity loop PID
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 0.5;
  motor.LPF_velocity.Tf = 0.007;

  motor.PID_velocity.output_ramp = 100;
  
  // angle loop PID
  // motor.P_angle.P = 100.0;
  motor.P_angle.P = 20.0;
  motor.LPF_angle.Tf = 0.005;

   // foc current control parameters (Arduino UNO/Mega)
  motor.PID_current_q.P = 2;
  motor.PID_current_q.I= 10;
  motor.PID_current_d.P= 2;
  motor.PID_current_d.I = 10;

  motor.LPF_current_q.Tf = 0.005;
  motor.LPF_current_d.Tf = 0.005;
  
    // Limits 
  motor.velocity_limit = 20.0; 
  motor.voltage_limit = 23.0;   // 12 Volt limit 
  motor.current_limit = 20.0;    // 2 Amp current limit


  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);



  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // motor.sensor_offset = motor.shaft_angle;
  motor.target = motor.shaft_angle;

  // motor.target = 4.5;

  // add target command T
  command.add('T', doTarget, "target current");

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

static uint32_t gSendDate = 0 ;
static uint32_t gSentCount1 = 0 ;
static uint32_t gReceivedCount1 = 0 ;

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();

    //     if (millis() - time_ms > 20){
    // PhaseCurrent_s current = current_sense.getPhaseCurrents();
    // Serial.print(current.a);
    // Serial.print("\t");
    // Serial.print(current.b);
    // Serial.print("\t");
    // Serial.print(current.c);
    // Serial.println("");
    // time_ms = millis();
    // }

//     if (gSendDate < millis ()) {
//     gSendDate += 1000 ;
//     // digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
//     CANFDMessage message ;
//     message.id = 0x7FF ;
//     message.len = 8 ;
//     message.data [0] = 0 ;
//     message.data [1] = 1 ;
//     message.data [2] = 2 ;
//     message.data [3] = 3 ;
//     message.data [4] = 4 ;
//     message.data [5] = 5 ;
//     message.data [6] = 6 ;
//     message.data [7] = 7 ;
// //--- tryToSendReturnStatusFD returns 0 if message has been successfully
// //    appended in transmit buffer.
// //  A not zero returned value contains an error code (see doc)
//     uint32_t sendStatus = fdcan1.tryToSendReturnStatusFD (message) ;
//     if (sendStatus == 0) {
//       gSentCount1 += 1 ;
//       Serial.print ("fdcan1 sent: ") ;
//       Serial.println (gSentCount1) ;
//     }

//   }
//   CANFDMessage messageFD ;
//   if (fdcan1.receiveFD0 (messageFD)) {
//     gReceivedCount1 += 1 ;
//     Serial.print ("fdcan1 received: ") ;
//     Serial.println (gReceivedCount1) ;
//   }

}
