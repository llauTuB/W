#include "config/lemlib_config.hpp"
#include "config/robot_config.hpp"
#include "globals/all_const.hpp"
#include "config/mcl_config.hpp"

// LemLib definitions

lemlib::TrackingWheel verticalTracker(&verticalEncoder, TRACKING_WHEELS, VERTICAL_OFFSET);
lemlib::TrackingWheel horizontalTracker(&horizontalEncoder, TRACKING_WHEELS, HORIZONTAL_OFFSET);

lemlib::OdomSensors sensors(&verticalTracker, nullptr, &horizontalTracker, nullptr, &imu);

lemlib::Drivetrain drivetrain(
    &leftDrive, 
    &rightDrive, 
    TRACK_WIDTH, 
    DRIVE_WHEELS, 
    RPM, 
    HORIZONTAL_DRIFT
);
lemlib::ControllerSettings drivePID(
    DRIVE_P, 
    DRIVE_I, 
    DRIVE_D, 
    DRIVE_ANTI_WINDUP, 
    DRIVE_SMALL_ERROR, 
    DRIVE_SMALL_TIMEOUT, 
    DRIVE_LARGE_ERROR, 
    DRIVE_LARGE_TIMEOUT, 
    DRIVE_ACCELERATION
);
lemlib::ControllerSettings turnPID(
    TURN_P, 
    TURN_I, 
    TURN_D, 
    TURN_ANTI_WINDUP, 
    TURN_SMALL_ERROR, 
    TURN_SMALL_TIMEOUT, 
    TURN_LARGE_ERROR, 
    TURN_LARGE_TIMEOUT, 
    TURN_ACCELERATION
);
lemlib::ExpoDriveCurve throttleCurve(5, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(5, 10, 1.021);
lemlib::Chassis robot(
    drivetrain, 
    drivePID, 
    turnPID, 
    sensors, 
    &throttleCurve, 
    &steerCurve,
    &filter
);