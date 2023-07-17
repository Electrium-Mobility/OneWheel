#include <StateMachine.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050.h>

#include "motorFOC.hpp"

// Init BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOT_A, MOT_B, MOT_C, MOT_EN);

// init motor Encoder & Interrupt 
// https://docs.simplefoc.com/hall_sensors
HallSensor hall_sensor = HallSensor(HALL_A, HALL_B, HALL_C, POLE_PAIRS);
// interrupt routine initialization
// Read Docs (here since we are using an STM32 all digital pins can be used
// as interrupt pins)
void doA(){hall_sensor.handleA();}
void doB(){hall_sensor.handleB();}
void doC(){hall_sensor.handleC();}

// init PID Controller
PIDController pid_stabalize{
  .P = Kp, .I = Ki, .D = Kd,
  .ramp = PID_Ramp, .limit = PID_Limit
};

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

// init State machine: https://github.com/jrullan/StateMachine
const int STATE_DELAY = 100; // TODO:determine
// int randomState = 0;
StateMachine machine = StateMachine();

// variables for time (integration)
uint32_t current_time = 0;
uint32_t prev_time = 0;

// state functions
void init_sequence();
void start_sequence();
void driving_loop();
void regen_brake();
void termination();

bool drive_transition();
bool regen_brake_transition();
bool termination_transition();
bool restart_transition();

// init States
State* Init_State = machine.addState(&init_sequence);
State* Starting_State = machine.addState(&start_sequence);
State* Driving_State = machine.addState(&driving_loop);
State* Regen_Brake_State = machine.addState(&regen_brake);
State* Termination_State = machine.addState(&termination);

void init_transitions()
{
    Starting_State->addTransition(&drive_transition, Driving_State);
    Driving_State->addTransition(&regen_brake_transition, Regen_Brake_State);
    Driving_State->addTransition(&termination_transition, Termination_State);
    Regen_Brake_State->addTransition(&termination_transition, Termination_State);
    Termination_State->addTransition(&restart_transition, Starting_State);
}

// Global control variables

void setup()
{
    Serial.begin(115200);
    _delay(1000);

    init_transitions();
}

void loop()
{
    imu->updateResult();
    current_time = micros();
    sampleFreq = (1000000.0f / (current_time - prev_time));
    prev_time = current_time;
    
    motor.loopFOC();
    motor.move();
    machine.run();
    delay(STATE_DELAY);
}

void init_sequence()
{
  // use internal pullups
  hall_sensor.pullup = Pullup::USE_INTERN;
  // maximal expected velocity
  hall_sensor.velocity_max = 1000; // 1000rad/s by default ~10,000 rpm
  // enable hall sensor hardware interrupts
  hall_sensor.init();
  hall_sensor.enableInterrupts(doA, doB, doC);
  // link motor with sensor
  motor.linkSensor(&hall_sensor);

  // driver config
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 6;
  driver.init();
  // linit motor with driver
  motor.linkDriver(&driver);
  
  // motor config
  motor.voltage_limit = 3;
  motor.controller = MotionControlType::torque;
  // voltage torque control mode 
  // TODO: do more reading on torque control:
  // https://docs.simplefoc.com/voltage_torque_mode
  motor.torque_controller = TorqueControlType::voltage;
  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();

  // init Pressure sensors
  pinMode(Pressure_1, INPUT);
  pinMode(Pressure_2, INPUT);

  machine.transitionTo(Starting_State);
}

void start_sequence()
{
    // read pressure sensors
    P1_Val = digitalRead(Pressure_1);
    P2_Val = digitalRead(Pressure_2);
    
    current_pitch = data->angleY;
}

bool drive_transition()
{
    // are both feet on?
    bool feet = P1_Val && P2_Val;
    // is tilt 0 or within +- 10 deg of 0 deg?
    bool tilt = abs(current_pitch) <= 10;

    return feet && tilt;
}

float threshold_checks()
{
    bool case1 = motor.target > float(KICK_BACK_THRESH)/100 * motor.voltage_limit;
    bool case2 = component_temp > HIGH_COMPONENT_TEMP;
    bool case3 = batteryVoltage > HIGH_BATTERY_VOLTAGE 
                 || batteryVoltage < LOW_BATTERY_VOLTAGE;

    if (case2 || case3)
    {
        return KICK_BACK_CONSTANT;
    }
    else if (case1)
    {
        // TODO: Determine kick back constant expirementally
        const float KICK_BACK_MULT_CONST {10};
        float voltageDifference = motor.target - float(KICK_BACK_THRESH)
                                  / 100 * motor.voltage_limit;
        float override = - KICK_BACK_MULT_CONST * voltageDifference;
        return override;
    }
    return 0;
}

void driving_loop()
{
    current_pitch = data->angleY;

    float x_tilt = data->angleX;
    // TODO: Use x_tilt for boost (entails changing pitchError)
    
    float pitchError = target_pitch - current_pitch;
    float voltageControl = pid_stabalize(pitchError);

    motor.target = voltageControl;
    
    target_pitch = threshold_checks();
}

bool regen_brake_transition()
{
    if (batteryPercentage < MAX_BATTERY_REGEN)
    {
        bool lowSpeed = abs(motor.shaft_velocity) < LOW_SPEED_THRESH;
        bool lowTargetVoltage = motor.target < LOW_VOLTAGE_TARGET;

        if (lowSpeed && lowTargetVoltage)
            return true;
        return false;
    }
    return false;
}

bool termination_transition()
{
    P1_Val = digitalRead(Pressure_1);
    P2_Val = digitalRead(Pressure_2);

    bool feet = P1_Val && P2_Val;
    if (feet) // both feet on
        return false;

    bool lowSpeed = abs(motor.shaft_velocity) < TERMINATE_LOW_SPEED;
    feet = P1_Val || P2_Val; // either feet is on

    return lowSpeed || !feet;
}

void regen_brake()
{
    // TODO: implment regen braking
    return ;
}

void termination()
{
    float currentTarget = motor.target;
    delay(TERMINATE_DELAY);
    for (; currentTarget > 0; currentTarget -= 1)
    {
        delay(100);
        motor.target = currentTarget;
    }
    currentTarget = 0;
    motor.target = 0;
}

bool restart_transition()
{
    // both feet off? 
    P1_Val = digitalRead(Pressure_1);
    P2_Val = digitalRead(Pressure_2);

    bool feetOff = !(P1_Val || P2_Val);

    // motor off
    bool motorOff = motor.target == 0;

    return feetOff && motorOff;
}
