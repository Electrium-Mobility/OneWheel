// Open loop motor control example
#include <SimpleFOC.h>
#include "./IMU/IMU.hpp"
#include <Wire.h>
#include <MPU6050.h>

#define MOT_A D1
#define MOT_B D2
#define MOT_C D3
#define MOT_EN D4

#define ENC_A 5
#define ENC_B 6

// PID variables
// TODO: experminatlly configure
#define Kp 0
#define Ki 0
#define Kd 0
#define PID_RAMP 100000
#define PID_LIMIT 7

float target_pitch = 0;
float current_pitch = 0;

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(D1, D2, D3, D4);

// create Encoder Instances https://docs.simplefoc.com/encoder
// TODO: test getting encoder reading from motor
Encoder encoder = Encoder(ENC_A, ENC_B, 500);

// Interrupt init
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// control algorithim
// pid for stabaliztaion


PIDController pid_stablize(.P = Kp, .I = Ki,
                           .D = Kd, .ramp = PID_RAMP,
                           .limit = PID_LIMIT);

//target variable
float target_velocity = 0;

// instantiate the commander
Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

MPU6050 mpu;
IMU* imu = new IMU(Wire, &mpu);
IMU_data* data = imu->getData();

void setup() {
  Serial.begin(115200);
  _delay(1000);

  // init encoders
  encoder.init();
  encoder.enableInterrupts(doA, doB);


  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // link the motor and sensor
  motor.linkSensor(&sensor);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 3;   // [V]
 
  // voltage torque control config
  motor.controller = MotionControlType::torque;
  motor.useMonitoring(Serial);

  // init motor hardware
  motor.init();
  motor.initFOC();

  // add target command T
  //command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
}

void loop() {

  imu->updateResult();
  current_pitch = data->angleY;

  float voltageControl = pid_stablize(target_pitch - currentPitch);

  motor.target = voltageControl;

  // user communication
  command.run();
}
