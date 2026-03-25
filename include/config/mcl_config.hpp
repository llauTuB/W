#pragma once
#include "lemlib/pose.hpp"
#include "lemlib/distancesensor.hpp"
#include "lemlib/chassis/particlefilter.hpp"
// MCL definitions (declared in globals.hpp)

extern lemlib::Pose distanceRightOffset;
extern lemlib::Pose distanceLeftOffset;
extern lemlib::Pose distanceFrontOffset;
extern lemlib::Pose distanceBackOffset;

extern cavalry::DistanceSensor distanceSensorRight;
extern cavalry::DistanceSensor distanceSensorLeft;
extern cavalry::DistanceSensor distanceSensorFront;
extern cavalry::DistanceSensor distanceSensorBack;

extern cavalry::ParticleFilterSettings particleFilterSettings;

extern cavalry::ParticleFilterSensors particleFilterSensors;

extern cavalry::ParticleFilter filter;