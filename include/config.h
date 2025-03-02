

#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// drive motors
#define IS_RED true

#define LEFT_FRONT_DRIVE -18
#define LEFT_MIDDLE_DRIVE -20
#define LEFT_BACK_DRIVE -19

#define RIGHT_FRONT_DRIVE 10
#define RIGHT_MIDDLE_DRIVE 8
#define RIGHT_BACK_DRIVE 9

// drive config
#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

// intake motors
#define INTAKE_PREROLLER 17
#define INTAKE_HOOKS -14

// wall stake motors
#define WALL_MOTOR 3

// pneumatics
#define DOINKER 'G'
#define CLAMP 'B'
#define RUSH 'H'
#define LIFT 'C' // h

// sensors
#define WALL_ROTATION 6
#define IMU 12
#define VERTICAL_ODOM 16
#define HORIZONTAL_ODOM -13

// ring hold
#define RING_DISTANCE 7
//**IN MILLIMETERS** the value that the ring must be closer than to be detected
#define RING_DISTANCE_THRESHOLD 100
#define RING_PROXIMITY 100

// color sort
#define RING_COLOR 2
//*IN MILLISECONDS* the time that it takes for the ring to reach the top of the hooks from when the color sensor detects it*/
#define COLOR_TIME 500
inline pros::Controller master(pros::E_CONTROLLER_MASTER);

inline pros::Motor left_front_drive(LEFT_FRONT_DRIVE);
inline pros::Motor left_middle_drive(LEFT_MIDDLE_DRIVE);
inline pros::Motor left_back_drive(LEFT_BACK_DRIVE);
inline pros::Motor right_front_drive(RIGHT_FRONT_DRIVE);
inline pros::Motor right_middle_drive(RIGHT_MIDDLE_DRIVE);
inline pros::Motor right_back_drive(RIGHT_BACK_DRIVE);

inline pros::MotorGroup left_mg({LEFT_FRONT_DRIVE, LEFT_MIDDLE_DRIVE, LEFT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_DRIVE, RIGHT_MIDDLE_DRIVE, RIGHT_BACK_DRIVE}, pros::MotorGearset::blue);

inline pros::Rotation wall_rotation(WALL_ROTATION);
inline pros::Motor wall_motor(WALL_MOTOR);

inline pros::Motor intake_hooks(INTAKE_HOOKS);
inline pros::Motor intake_preroller(INTAKE_PREROLLER);

// inline pros::Imu imu(IMU);

inline pros::adi::Pneumatics clamp(CLAMP, false);
inline pros::adi::Pneumatics doinker(DOINKER, false);
inline pros::adi::Pneumatics rush(RUSH, false);
inline pros::adi::Pneumatics lift(LIFT, true);

inline pros::Distance ring_distance(RING_DISTANCE);
inline pros::Optical ring_color(RING_COLOR);

inline pros::Rotation vertical_odom(VERTICAL_ODOM);
inline pros::Rotation horizontal_odom(HORIZONTAL_ODOM);
