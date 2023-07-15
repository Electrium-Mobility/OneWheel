#include "motorFOC.hpp"

// Init BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDrivers3PWM(D1, D2, D3, D4);

// init motor Encoder & Interrupt 
// https://docs.simplefoc.com/hall_sensors
HallSensor hall_sensor = HallSensor(HALL_A, HALL_B, HALL_C, POLE_PAIRS);
// use internal pullups
sensor.pullup = Pullup::USE_INTERN;
// maximal expected velocity
sensor.velocity_max = 1000; // 1000rad/s by default ~10,000 rpm
// interrupt routine initialization
// Read Docs (here since we are using an STM32 all digital pins can be used
// as interrupt pins)
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

// init PID Controller
PIDController pid_stabalize(
  .P = Kp, .I = Ki, .D = Kd,
  .ramp = PID_Ramp, .limit = PID_Limit
);

// IMU Sensor
MPU6050 mpu;
IMU* imu = new IMU(Wire, &mpu);
IMU_data* data = imu->getData();
float target_pitch = 0;
float current_pitch = 0;

// Pressure Sensors 
// TODO: Determine if its LOW off or HIGH off
int P1_Val{LOW}, P2_Val{LOW};

// temp Battery Reading
// TODO: receive battery reading
float batteryVoltage = 4;
int batteryPercentage = 80;

// temp temperature reading
float component_temp = 70; // [deg C]