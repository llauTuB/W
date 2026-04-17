#include "lemlib/chassis/particlefilter.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace daniyar {

ParticleFilter::ParticleFilter(ParticleFilterSensors sensors, ParticleFilterSettings settings, bool enabled) : settings(settings), randomGenerator(std::random_device{}()) {
    set_enabled(enabled);

    NUM_PARTICLES = settings.numParticles;
    particles.resize(NUM_PARTICLES);
    oldParticles.resize(NUM_PARTICLES);
    weights.resize(NUM_PARTICLES);

    for (auto& particle : particles) {
        particle = {0, 0};
    }
  
    for (auto sensor : sensors.distanceSensors) {
        if (sensor != nullptr) {
            this->sensors.push_back(sensor);
        }
    }
    
    this->imu = sensors.imu;
    this->verticalWheel = sensors.verticalWheel;
    this->horizontalWheel = sensors.horizontalWheel;
}

void ParticleFilter::init_uniform(float xMin, float xMax, float yMin, float yMax) {
    std::uniform_real_distribution<float> xDist(xMin, xMax);
    std::uniform_real_distribution<float> yDist(yMin, yMax);

    for (auto& particle : particles) {
        particle.first = xDist(randomGenerator);
        particle.second = yDist(randomGenerator);
    }
}

void ParticleFilter::init_normal(const lemlib::Pose& mean, float stdDevX, float stdDevY) {
    std::normal_distribution<float> xDist(mean.x, stdDevX);
    std::normal_distribution<float> yDist(mean.y, stdDevY);

    for (auto& particle : particles) {
        particle.first = xDist(randomGenerator);
        particle.second = yDist(randomGenerator);
    }
}

void ParticleFilter::update() {
    // Poll tracking wheels to get positional change
    float deltaOdomVertical = 0.0f;
    if (verticalWheel != nullptr) {
        deltaOdomVertical = verticalWheel->getDistanceTraveled() - lastOdomVertical;
        lastOdomVertical += deltaOdomVertical;
    }

    float deltaOdomHorizontal = 0.0f;
    if (horizontalWheel != nullptr) {
        deltaOdomHorizontal = horizontalWheel->getDistanceTraveled() - lastOdomHorizontal;
        lastOdomHorizontal += deltaOdomHorizontal;
    }

    float deltaHeading = imu->get_rotation() - lastImuHeading;
    lastImuHeading = imu->get_rotation();

    float currentTheta = prediction.theta + deltaHeading;

    // Update the motion distributions
    create_distributions(deltaOdomVertical, deltaOdomHorizontal, currentTheta);

    // Shift each particle by noisy motion to account for uncertainty in odometry
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        lemlib::Pose noisyMotion = get_noisy_motion();
        particles[i].first += noisyMotion.x;
        particles[i].second += noisyMotion.y;
    }

    // 1. Calculate true loop distance traveled
    float loopDistanceTraveled = std::hypot(deltaOdomVertical, deltaOdomHorizontal);
    
    // 2. Accumulate the tiny distance deltas over time
    distanceSinceLastUpdate += loopDistanceTraveled;
    
    // 3. Directly assign the total elapsed time
    timeSinceLastUpdate = pros::millis() - timeLastUpdate;

    // Only resample if enough distance or time has passed
    if (distanceSinceLastUpdate < settings.maxDistance && timeSinceLastUpdate < settings.maxTime) {
        float xSum = 0.0f, ySum = 0.0f;

        for (const auto& particle : particles) {
            xSum += particle.first;
            ySum += particle.second;
        }

        prediction.x = xSum / static_cast<float>(NUM_PARTICLES);
        prediction.y = ySum / static_cast<float>(NUM_PARTICLES);
        prediction.theta = currentTheta;

        return;
    }

    timeLastUpdate = pros::millis();
    distanceSinceLastUpdate = 0.0f;
    timeSinceLastUpdate = 0.0f;

    // Resample each distance sensor
    update_sensors();

    // Weight particles based on sensor readings and move any particles out of field
    float weightSum = weight_particles(currentTheta);

    if (weightSum <= 0.000001f) {
        std::cerr << "All weights are near ZERO - skipping resample to avoid crash" << std::endl;
        return;
    }

    // Resamples particles based on weights and updates prediction
    std::pair<double, double> estimate = resample_particles(weightSum);
    prediction.x = estimate.first;
    prediction.y = estimate.second;
    prediction.theta = currentTheta;
}

void ParticleFilter::add_sensor(DistanceSensor* sensor) {
    this->sensors.push_back(sensor);
}

void ParticleFilter::add_circle(Circle circle) { 
    circles.push_back(circle); 
}

void ParticleFilter::add_polygon(Polygon polygon) { 
    polygons.push_back(polygon); 
}

void ParticleFilter::set_starting_pose(const lemlib::Pose& pose, bool normal) {
    this->prediction = pose;
    
    if (normal) {
        init_normal(pose, settings.startRadius, settings.startRadius);
    } else {
        init_uniform(-DIST_WALL_FROM_ZERO, DIST_WALL_FROM_ZERO,
                    -DIST_WALL_FROM_ZERO, DIST_WALL_FROM_ZERO);
    }

    this->startPoseInitialized = true;
}

void ParticleFilter::set_enabled(bool enabled) {
    this->enabled = enabled;
}

lemlib::Pose ParticleFilter::get_prediction() const { 
    return prediction; 
}

std::vector<std::pair<float, float>> ParticleFilter::get_particles() const {
    return std::vector<std::pair<float, float>>(particles.begin(), particles.end());
}

std::vector<float> ParticleFilter::get_weights() const {
    return std::vector<float>(weights.begin(), weights.end());
}

const std::vector<Circle>& ParticleFilter::get_circles() const { 
    return circles; 
}

const std::vector<Polygon>& ParticleFilter::get_polygons() const { 
    return polygons; 
}

bool ParticleFilter::is_initialized() {
    return this->startPoseInitialized;
}

bool ParticleFilter::is_enabled() {
    return this->enabled && this->is_initialized();
}


void ParticleFilter::create_distributions(float deltaOdomVertical, float deltaOdomHorizontal, float currentTheta) {
    const float BASE_VERTICAL_NOISE = 0.3f; 
    const float BASE_HORIZONTAL_NOISE = 0.3f;

    float vNoiseSpread = BASE_VERTICAL_NOISE + (settings.verticalNoise * std::abs(deltaOdomVertical));
    verticalDistribution = std::uniform_real_distribution<>(deltaOdomVertical - vNoiseSpread, 
                                                            deltaOdomVertical + vNoiseSpread);

    float hNoiseSpread = BASE_HORIZONTAL_NOISE + (settings.horizontalNoise * std::abs(deltaOdomHorizontal));
    horizontalDistribution = std::uniform_real_distribution<>(deltaOdomHorizontal - hNoiseSpread, 
                                                              deltaOdomHorizontal + hNoiseSpread);

    angularDistribution = std::uniform_real_distribution<>(currentTheta - settings.angularNoise, 
                                                           currentTheta + settings.angularNoise);
}

void ParticleFilter::update_sensors() {
    for (auto&& sensor : sensors) {
        sensor->update();
    }
}

float ParticleFilter::weight_particles(float currentTheta) {
    // --- 1. PROBABILISTIC ODOMETRY GATING ---
    
    // First, find our "Deterministic Belief" by averaging the particles 
    // after they were shifted by the tracking wheels.
    float sumX = 0.0f, sumY = 0.0f;
    for (const auto& p : particles) {
        sumX += p.first;
        sumY += p.second;
    }
    lemlib::Pose expectedPose(sumX / NUM_PARTICLES, sumY / NUM_PARTICLES, currentTheta);

    std::vector<DistanceSensor*> original_sensors = sensors;
    std::vector<DistanceSensor*> valid_sensors;

    for (auto sensor : sensors) {
        if (sensor->is_disabled()) continue;
        
        // Evaluate the probability of the physical sensor reading IF the robot 
        // was sitting exactly at the expected Odometry pose.
        auto expected_prob = sensor->p(expectedPose, circles, polygons);
        
        // If the probability is greater than a microscopic amount, the reading is 
        // believable. If it's near zero, it hit a robot/gamepiece and we reject it!
        if (expected_prob.has_value() && expected_prob.value() > 0.00001f) {
            valid_sensors.push_back(sensor);
        }
    }

    // Temporarily swap to only use the unblocked sensors for the particle evaluation
    sensors = valid_sensors;

    // --- 2. PARTICLE WEIGHTING ---

    float weightSum = 0.0f;
    for (size_t i = 0; i < NUM_PARTICLES; ++i) {
        if (out_of_field(particles[i])) {
            particles[i].first = fieldDistribution(randomGenerator);
            particles[i].second = fieldDistribution(randomGenerator);
        }

        lemlib::Pose particlePose(particles[i].first, particles[i].second, currentTheta);
        weights[i] = weight_particle(particlePose);
        weightSum += weights[i];
    }

    // Restore the original sensor list for the next loop
    sensors = original_sensors;

    return weightSum;
}

std::pair<double, double> ParticleFilter::resample_particles(float weightSum) {
    const double avgWeight = weightSum / static_cast<double>(NUM_PARTICLES);
    std::uniform_real_distribution<float> distribution(0.0f, avgWeight);
    const float randWeight = distribution(randomGenerator);

    oldParticles = particles; // Safe vector copy assignment

    size_t j = 0;
    float cumulativeWeight = weights[0]; // Initialize with first weight
    float xSum = 0.0f, ySum = 0.0f;

    for (size_t i = 0; i < NUM_PARTICLES; i++) {
        const double targetWeight = randWeight + i * avgWeight;

        // Advance j until we cover the target weight
        while (targetWeight > cumulativeWeight && j < NUM_PARTICLES - 1) {
            j++;
            cumulativeWeight += weights[j];
        }

        particles[i] = oldParticles[j];
        xSum += particles[i].first;
        ySum += particles[i].second;
    }

    return {xSum / NUM_PARTICLES, ySum / NUM_PARTICLES};
}

float ParticleFilter::weight_particle(const lemlib::Pose& particle) {
    float totalWeight = 1.0;

    for (const auto sensor: sensors) {
        if (sensor->is_disabled()) {
            continue;
        }
        
        auto weight = sensor->p(particle, circles, polygons); 
        if (weight.has_value() && std::isfinite(weight.value())) {
            totalWeight = totalWeight * weight.value();
        }
    }

    // Safety clamp: guarantees weight is never exactly 0.0
    return std::max(totalWeight, 0.0000001f);
}

lemlib::Pose ParticleFilter::get_noisy_motion() {
    const double verticalNoise = verticalDistribution(randomGenerator);
    const double horizontalNoise = horizontalDistribution(randomGenerator);
    const double angularNoise = -angularDistribution(randomGenerator) * (M_PI / 180.0);

    return lemlib::Pose(horizontalNoise, verticalNoise).rotate(angularNoise);
}

float ParticleFilter::calculate_norm(const lemlib::Pose& pose) {
    return std::sqrt(pose.x * pose.x + pose.y * pose.y);
}

bool ParticleFilter::out_of_field(const std::pair<float, float>& particle) {
    return particle.first > DIST_WALL_FROM_ZERO 
        || particle.first < -DIST_WALL_FROM_ZERO 
        || particle.second < -DIST_WALL_FROM_ZERO 
        || particle.second > DIST_WALL_FROM_ZERO;
}

} // namespace daniyar