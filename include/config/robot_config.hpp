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

extern pros::Controller master;

extern std::vector<std::int8_t> rightDrivePorts;
extern pros::MotorGroup rightDrive;
extern std::vector<std::int8_t> leftDrivePorts;
extern pros::MotorGroup leftDrive;

extern pros::Motor intakeStage1;
extern pros::Motor intakeStage2;
extern pros::Motor intakeStage3;

extern pros::IMU imu;
extern pros::Rotation verticalEncoder;
extern pros::Rotation horizontalEncoder;

extern pros::Distance distance_right;
extern pros::Distance distance_left;
extern pros::Distance distance_back;
extern pros::Distance distance_front;

extern pros::adi::Pneumatics loader;
extern pros::adi::Pneumatics descore;
extern pros::adi::Pneumatics middle_descore;
extern pros::adi::Pneumatics upScore;
extern pros::adi::Pneumatics encoder_upper;
extern pros::adi::Pneumatics intake_upper;
