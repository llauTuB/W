#include "config/lemlib_config.hpp"
#include "config/robot_config.hpp"
#include "globals/all_const.hpp"
#include "config/mcl_config.hpp"

// LemLib definitions

lemlib::TrackingWheel verticalTracker(&verticalEncoder, 2, VERTICAL_OFFSET);
lemlib::TrackingWheel horizontalTracker(&horizontalEncoder, 2.75, HORIZONTAL_OFFSET);

lemlib::OdomSensors sensors(nullptr, &verticalTracker, &horizontalTracker, nullptr, &imu);

lemlib::Drivetrain drivetrain(
    &leftDrive, 
    &rightDrive, 
    TRACK_WIDTH, 
    DRIVE_WHEELS, 
    RPM, 
    HORIZONTAL_DRIFT
);

lemlib::ControllerSettings drivePID (9.1, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            67, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

lemlib::ControllerSettings turnPID(5.9, // proportional gain (kP)
                                             0.0, // integral gain (kI)
                                             52, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve throttleCurve(5, 10, 1.03);
lemlib::ExpoDriveCurve steerCurve(5, 10, 1.025);

lemlib::Chassis robot(
    drivetrain, 
    drivePID, 
    turnPID, 
    sensors,
    &throttleCurve, 
    &steerCurve,
    &filter
);