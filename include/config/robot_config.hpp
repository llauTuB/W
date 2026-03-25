#pragma once
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

extern std::vector<std::int8_t> rightDrivePorts;
extern pros::MotorGroup rightDrive;

extern std::vector<std::int8_t> leftDrivePorts;
extern pros::MotorGroup leftDrive;

extern pros::Motor intakeHigh1;
extern pros::Motor intakeHigh2;
extern pros::Motor intakeLow;
extern pros::Controller master;


extern pros::IMU imu;
extern pros::Rotation verticalEncoder;
extern pros::Rotation horizontalEncoder;
extern pros::Controller master;

extern pros::adi::Pneumatics loader;
extern pros::adi::Pneumatics descore;
extern pros::adi::Pneumatics doublePark;
extern pros::adi::Pneumatics upScore;
