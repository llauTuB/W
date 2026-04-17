#pragma once
#include "lemlib/pose.hpp"
#include "lemlib/distancesensor.hpp"
#include "lemlib/chassis/particlefilter.hpp"
// MCL definitions (declared in globals.hpp)

extern lemlib::Pose distanceRightOffset;
extern lemlib::Pose distanceLeftOffset;
extern lemlib::Pose distanceFrontOffset;
extern lemlib::Pose distanceBackOffset;

extern daniyar::DistanceSensor distanceSensorRight;
extern daniyar::DistanceSensor distanceSensorLeft;
extern daniyar::DistanceSensor distanceSensorFront;
extern daniyar::DistanceSensor distanceSensorBack;

extern daniyar::ParticleFilterSettings particleFilterSettings;

extern daniyar::ParticleFilterSensors particleFilterSensors;

extern daniyar::ParticleFilter filter;