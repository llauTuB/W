#include "config/robot_config.hpp"
#include "globals/ports.hpp"
#include "globals/robot_const.hpp"

std::vector<std::int8_t> rightDrivePorts = {ports::DRIVE_RIGHT_FRONT, ports::DRIVE_RIGHT_MIDDLE, ports::DRIVE_RIGHT_BACK};
pros::MotorGroup rightDrive(rightDrivePorts, DRIVE_GEARSET);

std::vector<std::int8_t> leftDrivePorts = {ports::DRIVE_LEFT_FRONT, ports::DRIVE_LEFT_MIDDLE, ports::DRIVE_LEFT_BACK};
pros::MotorGroup leftDrive (leftDrivePorts, DRIVE_GEARSET);
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor intakeHigh1(ports::INTAKE_HIGH1_PORT, INTAKE_GEARSET);
pros::Motor intakeHigh2(ports::INTAKE_HIGH2_PORT, INTAKE_GEARSET);
pros::Motor intakeLow(ports::INTAKE_LOW_PORT, INTAKE_GEARSET);

pros::IMU imu(ports::IMU_PORT);
pros::Rotation verticalEncoder(ports::VERTICAL_ENCODER_PORT);
pros::Rotation horizontalEncoder(ports::HORIZONTAL_ENCODER_PORT);

pros::adi::Pneumatics loader(ports::LOADER_PORT, false);
pros::adi::Pneumatics descore(ports::DESCORE_PORT, false);

pros::adi::Pneumatics upScore(ports::UPSCORE_PORT, false);
pros::adi::Pneumatics doublePark(ports::DOUBLE_PARK_PORT, false);