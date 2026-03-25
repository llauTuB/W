#pragma once

#include "pros/imu.hpp"
#include "lemlib/distancesensor.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/geom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

#include <vector>
#include <random>

namespace cavalry {

class ParticleFilterSettings {
public:
    ParticleFilterSettings(int numParticles,
                           float verticalNoise,
                           float horizontalNoise,
                           float angularNoise,
                           float maxDistance,
                           float maxTime,
                           float startRadius)
        : numParticles(numParticles),
          verticalNoise(verticalNoise),
          angularNoise(angularNoise),
          horizontalNoise(horizontalNoise),
          maxDistance(maxDistance),
          maxTime(maxTime),
          startRadius(startRadius) {}
    int numParticles;
    float verticalNoise;
    float horizontalNoise;
    float angularNoise;
    float maxDistance;
    float maxTime;
    float startRadius;
};

class ParticleFilterSensors {
public:
    ParticleFilterSensors(pros::IMU* imu, 
                        std::vector<DistanceSensor*> distanceSensors,
                        lemlib::TrackingWheel* verticalWheel = nullptr,
                        lemlib::TrackingWheel* horizontalWheel = nullptr)
        : imu(imu), distanceSensors(std::move(distanceSensors)), verticalWheel(verticalWheel), horizontalWheel(horizontalWheel) {}

    pros::IMU* imu;
    std::vector<DistanceSensor*> distanceSensors;
    lemlib::TrackingWheel* verticalWheel;
    lemlib::TrackingWheel* horizontalWheel;
};


class ParticleFilter {
private:
    int NUM_PARTICLES = 0;

    std::vector<std::pair<float, float>> particles;
    std::vector<std::pair<float, float>> oldParticles;
    std::vector<float> weights;

    pros::IMU* imu = nullptr;
    std::vector<DistanceSensor*> sensors;
    lemlib::TrackingWheel* verticalWheel = nullptr;
    lemlib::TrackingWheel* horizontalWheel = nullptr;
    
    std::vector<Circle> circles;
    std::vector<Polygon> polygons;

    lemlib::Pose prediction = lemlib::Pose(0, 0, 0);

    std::uniform_real_distribution<> fieldDistribution {-DIST_WALL_FROM_ZERO, DIST_WALL_FROM_ZERO};
    std::uniform_real_distribution<> verticalDistribution;
    std::uniform_real_distribution<> horizontalDistribution;
    std::uniform_real_distribution<> angularDistribution;

    std::mt19937 randomGenerator;
    ParticleFilterSettings settings;

    float lastOdomVertical = 0.0f;
    float lastOdomHorizontal = 0.0f;
    float lastImuHeading = 0.0f;
    float distanceSinceLastUpdate = 0.0f;
    float timeSinceLastUpdate = 0.0f;
    float timeLastUpdate = 0.0f;
    bool startPoseInitialized = false;
    bool enabled = false;

public:
    ParticleFilter(ParticleFilterSensors sensors, ParticleFilterSettings settings, bool enabled);

    void init_uniform(float xMin, float xMax, float yMin, float yMax);

    void init_normal(const lemlib::Pose& mean, float stdDevX, float stdDevY);

    void update();

    void add_sensor(DistanceSensor* sensor);

    void add_circle(Circle circle);

    void add_polygon(Polygon polygon);

    void set_starting_pose(const lemlib::Pose& pose, bool normal);

    void set_enabled(bool enabled);

    lemlib::Pose get_prediction() const;

    std::vector<std::pair<float, float>> get_particles() const;

    std::vector<float> get_weights() const;

    const std::vector<Circle>& get_circles() const;

    const std::vector<Polygon>& get_polygons() const;

    bool is_initialized();

    bool is_enabled();

private:
    void create_distributions(float deltaOdomVertical, float deltaOdomHorizontal, float currentTheta);

    void update_sensors();

    float weight_particles(float currentTheta);

    std::pair<double, double> resample_particles(float weightSum);

    float weight_particle(const lemlib::Pose& particle);

    lemlib::Pose get_noisy_motion();

    float calculate_norm(const lemlib::Pose& pose);

    bool out_of_field(const std::pair<float, float>& particle);
};
} // namespace lemlib