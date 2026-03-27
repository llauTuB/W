#include "config/robot_config.hpp"
#include "globals/ports.hpp"
#include "globals/robot_const.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

std::vector<std::int8_t> rightDrivePorts = {ports::DRIVE_RIGHT_FRONT, ports::DRIVE_RIGHT_MIDDLE, ports::DRIVE_RIGHT_BACK};
pros::MotorGroup rightDrive(rightDrivePorts, DRIVE_GEARSET);

std::vector<std::int8_t> leftDrivePorts = {ports::DRIVE_LEFT_FRONT, ports::DRIVE_LEFT_MIDDLE, ports::DRIVE_LEFT_BACK};
pros::MotorGroup leftDrive (leftDrivePorts, DRIVE_GEARSET);


pros::Motor intakeStage1(ports::INTAKE_STAGE1_PORT, INTAKE_GEARSET);
pros::Motor intakeStage2(ports::INTAKE_STAGE2_PORT, INTAKE_GEARSET);
pros::Motor intakeStage3(ports::INTAKE_STAGE3_PORT, INTAKE_GEARSET);

pros::IMU imu(ports::IMU_PORT);
pros::Rotation verticalEncoder(ports::VERTICAL_ENCODER_PORT);
pros::Rotation horizontalEncoder(ports::HORIZONTAL_ENCODER_PORT);

pros::Distance distance_right(ports::DISTANCE_RIGHT_PORT);
pros::Distance distance_left(ports::DISTANCE_LEFT_PORT);
pros::Distance distance_back(ports::DISTANCE_BACK_PORT);
pros::Distance distance_front(ports::DISTANCE_FRONT_PORT);

pros::adi::Pneumatics loader(ports::LOADER_PORT, false);
pros::adi::Pneumatics descore(ports::DESCORE_PORT, false);
pros::adi::Pneumatics upScore(ports::UPSCORE_PORT, false);
pros::adi::Pneumatics intake_upper(ports::INTAKE_UPPER_PORT, false);
pros::adi::Pneumatics encoder_upper(ports::ENCODER_UPPER_PORT, true);
pros::adi::Pneumatics middle_descore(ports::MIDDLE_DESCORE_PORT, false);