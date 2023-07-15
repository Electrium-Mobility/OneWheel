#include <StateMachine.h>

#include "motorFOC.hpp"

// init State machine: https://github.com/jrullan/StateMachine
const int STATE_DELAY = 100; // TODO:determine
// int randomState = 0;
StateMachine machine = StateMachine();

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
    motor.loopFOC();
    motor.move();
    machine.run();
    delay(STATE_DELAY);
}

void init_sequence()
{
  // enable hall sensor hardware interrupts
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  // link motor with sensor
  motor.linkSensor(&sensor);

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
    
    imu->updateResult();
    currentPitch = data->angleY;
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
    bool case1 = motor.target > KICK_BACK_THRESH(float)/100 * motor.voltage_limit;
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
        float voltageDifference = motor.target - KICK_BACK_THRESH(float)
                                  / 100 * motor.voltage_limit;
        float override = - KICK_BACK_MULT_CONST * voltageDifference;
        return override;
    }
    return 0;
}

void driving_loop()
{
    imu->updateResult();
    current_pitch = data->angleY;

    float x_tilt = data->angleX;
    // TODO: Use x_tilt for boost (entails changing pitchError)
    
    float pitchError = target_pitch - current_pitch;
    float voltageControl = pid_stabalize(pitchError);

    motor.target = voltageControl;
    
    target_pitch = threshold_checks()
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