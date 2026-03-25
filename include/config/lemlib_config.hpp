#pragma once
#include "lemlib/chassis/chassis.hpp"


extern lemlib::TrackingWheel horizontalTracker;
extern lemlib::TrackingWheel verticalTracker;

extern lemlib::Drivetrain drivetrain;

extern lemlib::ControllerSettings drivePID;
extern lemlib::ControllerSettings turnPID;

extern lemlib::OdomSensors sensors;

extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;

extern lemlib::Chassis robot;