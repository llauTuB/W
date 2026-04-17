#pragma once
#include "pros/misc.h"
#include "lemlib/chassis/trackingWheel.hpp"

// Controller bindings

constexpr auto INTAKE_STATE_LOWSCORE = pros::E_CONTROLLER_DIGITAL_L1;
constexpr auto INTAKE_STATE_MIDDLESCORE = pros::E_CONTROLLER_DIGITAL_L2;
constexpr auto INTAKE_STATE_LOAD = pros::E_CONTROLLER_DIGITAL_R2;

constexpr auto LOADER_TOGGLE = pros::E_CONTROLLER_DIGITAL_Y;
constexpr auto INTAKE_UPPER_TOGGLE = pros::E_CONTROLLER_DIGITAL_DOWN;
constexpr auto UPSCORE_DESCORE_TOGGLE = pros::E_CONTROLLER_DIGITAL_R1;
constexpr auto MIDDLE_DESCORE_TOGGLE = pros::E_CONTROLLER_DIGITAL_B;
constexpr auto ENCODER_UPPER_TOGGLE = pros::E_CONTROLLER_DIGITAL_LEFT;


constexpr auto FILTER_TOGGLE = pros::E_CONTROLLER_DIGITAL_UP;
constexpr auto FLIP_IMU_TOGGLE = pros::E_CONTROLLER_DIGITAL_X;

// Drivetrain constants
constexpr auto TRACK_WIDTH = 13.5;
constexpr auto DRIVE_WHEELS = lemlib::Omniwheel::NEW_325;
constexpr auto RPM = 450;
constexpr auto HORIZONTAL_DRIFT = 2;

// Motor gearsets
constexpr auto DRIVE_GEARSET = pros::MotorGearset::blue;
constexpr auto INTAKE_GEARSET = pros::MotorGearset::blue;


// Odometry constants
constexpr auto TRACKING_WHEELS = lemlib::Omniwheel::NEW_275;
constexpr auto VERTICAL_OFFSET = -2.5;
constexpr auto HORIZONTAL_OFFSET = -1.5;