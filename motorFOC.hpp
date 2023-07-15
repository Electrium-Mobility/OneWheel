#ifndef MOTOR_FOC_HPP
#define MOTOR_FOC_HPP

#include "Arduino.h"
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050.h>
#include "IMU.hpp"

// Defines
// Motor Pins
#define MOT_A 1
#define MOT_B 2
#define MOT_C 3
#define MOT_EN 4

#define POLE_PAIRS 11

// Hall Sensor Pins
#define HALL_A 5
#define HALL_B 6
#define HALL_C 7

// PID Constants
#define Kp 0
#define Ki 0
#define Kd 0
#define PID_Ramp 100000
#define PID_Limit 7

// Pressure Sensor Init
#define Pressure_1 8
#define Pressure_2 9

// Kick back Constant
#define KICK_BACK_THRESH 80

//  regen constants
#define MAX_BATTERY_REGEN 95
// TODO: determine expirementally these constants
#define LOW_SPEED_THRESH 20 
#define LOW_VOLTAGE_TARGET 0.2

// threshold checks constants
#define LOW_BATTERY_VOLTAGE 2.8
#define HIGH_BATTERY_VOLTAGE 4.3

#define HIGH_COMPONENT_TEMP 90

#define KICK_BACK_CONSTANT 20

// termination constants
#define TERMINATE_LOW_SPEED 50

// Termination wait time
#define TERMINATE_DELAY 3000 // 3 seconds

#endif 