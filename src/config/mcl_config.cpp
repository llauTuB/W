#include "config/mcl_config.hpp"
#include "globals/all_const.hpp"
#include "config/robot_config.hpp"
#include "config/lemlib_config.hpp"

lemlib::Pose distanceRightOffset(DISTANCE_RIGHT_OFFSET_X, DISTANCE_RIGHT_OFFSET_Y, DISTANCE_RIGHT_OFFSET_T);
lemlib::Pose distanceLeftOffset(DISTANCE_LEFT_OFFSET_X, DISTANCE_LEFT_OFFSET_Y, DISTANCE_LEFT_OFFSET_T);
lemlib::Pose distanceFrontOffset(DISTANCE_FRONT_OFFSET_X, DISTANCE_FRONT_OFFSET_Y, DISTANCE_FRONT_OFFSET_T);
lemlib::Pose distanceBackOffset(DISTANCE_BACK_OFFSET_X, DISTANCE_BACK_OFFSET_Y, DISTANCE_BACK_OFFSET_T);

daniyar::DistanceSensor distanceSensorRight(ports::DISTANCE_RIGHT_PORT, distanceRightOffset, DISTANCE_RIGHT_CONSTANT);
daniyar::DistanceSensor distanceSensorLeft(ports::DISTANCE_LEFT_PORT, distanceLeftOffset, DISTANCE_LEFT_CONSTANT);
daniyar::DistanceSensor distanceSensorFront(ports::DISTANCE_FRONT_PORT, distanceFrontOffset, DISTANCE_FRONT_CONSTANT);
daniyar::DistanceSensor distanceSensorBack(ports::DISTANCE_BACK_PORT, distanceBackOffset, DISTANCE_BACK_CONSTANT);

daniyar::ParticleFilterSettings particleFilterSettings(
    NUM_PARTICLES,
    1.0,
    1.0,
    1.0,
    1.0,
    50.0,
    20.0
);

daniyar::ParticleFilterSensors particleFilterSensors(
    &imu, 
    {&distanceSensorRight, &distanceSensorLeft, &distanceSensorFront, &distanceSensorBack},
    nullptr,
    &horizontalTracker
);

daniyar::ParticleFilter filter(particleFilterSensors, particleFilterSettings, true);
