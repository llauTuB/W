#include "config/mcl_config.hpp"
#include "globals/all_const.hpp"
#include "config/robot_config.hpp"
#include "config/lemlib_config.hpp"

// MCL definitions (declared in globals.hpp)
lemlib::Pose distanceRightOffset(DISTANCE_RIGHT_OFFSET_X, DISTANCE_RIGHT_OFFSET_Y, DISTANCE_RIGHT_OFFSET_T);
lemlib::Pose distanceLeftOffset(DISTANCE_LEFT_OFFSET_X, DISTANCE_LEFT_OFFSET_Y, DISTANCE_LEFT_OFFSET_T);
lemlib::Pose distanceFrontOffset(DISTANCE_FRONT_OFFSET_X, DISTANCE_FRONT_OFFSET_Y, DISTANCE_FRONT_OFFSET_T);
lemlib::Pose distanceBackOffset(DISTANCE_BACK_OFFSET_X, DISTANCE_BACK_OFFSET_Y, DISTANCE_BACK_OFFSET_T);

cavalry::DistanceSensor distanceSensorRight(ports::DISTANCE_RIGHT_PORT, distanceRightOffset, DISTANCE_RIGHT_CONSTANT);
cavalry::DistanceSensor distanceSensorLeft(ports::DISTANCE_LEFT_PORT, distanceLeftOffset, DISTANCE_LEFT_CONSTANT);
cavalry::DistanceSensor distanceSensorFront(ports::DISTANCE_FRONT_PORT, distanceFrontOffset, DISTANCE_FRONT_CONSTANT);
cavalry::DistanceSensor distanceSensorBack(ports::DISTANCE_BACK_PORT, distanceBackOffset, DISTANCE_BACK_CONSTANT);

cavalry::ParticleFilterSettings particleFilterSettings(
    NUM_PARTICLES,
    1.0,
    1.0,
    3.0,
    2.0,
    1000.0,
    5.0
);

cavalry::ParticleFilterSensors particleFilterSensors(
    &imu, 
    {&distanceSensorRight, &distanceSensorLeft, &distanceSensorFront, &distanceSensorBack},
    &verticalTracker,
    &horizontalTracker
);

cavalry::ParticleFilter filter(particleFilterSensors, particleFilterSettings, true);
