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

lemlib::ControllerSettings drivePID (12, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            100, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

lemlib::ControllerSettings turnPID(6.05, // proportional gain (kP)
                                             0.0, // integral gain (kI)
                                             48, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
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