#include "lemlib/chassis/particlefilter.hpp"
#include <iostream>

namespace cavalry {

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

    // // Estimate distance traveled using noisy motion to account for uncertainty
    // const lemlib::Pose distanceApproximation = get_noisy_motion();
    // distanceSinceLastUpdate += calculate_norm(distanceApproximation);
    // timeSinceLastUpdate += pros::millis() - timeLastUpdate;
    
    float loopDistanceTraveled = std::hypot(deltaOdomVertical, deltaOdomHorizontal);
    // 2. Accumulate the tiny distance deltas over time
    distanceSinceLastUpdate += loopDistanceTraveled;
    // 3. Directly assign the total elapsed time (DO NOT use +=)
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

    if (weightSum == 0.0f) {
        std::cerr << "All weights are ZERO" <<  std::endl;
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

// void ParticleFilter::create_distributions(float deltaOdomVertical, float deltaOdomHorizontal, float currentTheta) {
//     if (deltaOdomVertical == 0) {
//         verticalDistribution = std::uniform_real_distribution<>(-settings.verticalNoise, settings.verticalNoise);
//     }
//     else {
//         verticalDistribution = std::uniform_real_distribution<>(deltaOdomVertical - settings.verticalNoise * deltaOdomVertical, 
//                                                             deltaOdomVertical + settings.verticalNoise * deltaOdomVertical);
//     }

//     if (deltaOdomHorizontal == 0) {
//         horizontalDistribution = std::uniform_real_distribution<>(-settings.horizontalNoise, settings.horizontalNoise);
//     }
//     else {
//         horizontalDistribution = std::uniform_real_distribution<>(deltaOdomHorizontal - settings.horizontalNoise * deltaOdomHorizontal, 
//                                                             deltaOdomHorizontal + settings.horizontalNoise * deltaOdomHorizontal);
//     }

//     angularDistribution = std::uniform_real_distribution<>(currentTheta - settings.angularNoise, 
//                                                         currentTheta + settings.angularNoise);
// }


void ParticleFilter::create_distributions(float deltaOdomVertical, float deltaOdomHorizontal, float currentTheta) {
    // 1. Define a minimum baseline noise so the filter always has some uncertainty.
    // (Recommendation: Move these into ParticleFilterSettings in particlefilter.hpp for easier tuning later)
    const float BASE_VERTICAL_NOISE = 0.5f; 
    const float BASE_HORIZONTAL_NOISE = 0.5f;

    // 2. Calculate total spread: Base Noise + (Proportional Multiplier * Absolute Movement)
    // Note: settings.verticalNoise and horizontalNoise now act as proportional multipliers.
    float vNoiseSpread = BASE_VERTICAL_NOISE + (settings.verticalNoise * std::abs(deltaOdomVertical));
    verticalDistribution = std::uniform_real_distribution<>(deltaOdomVertical - vNoiseSpread, 
                                                            deltaOdomVertical + vNoiseSpread);

    float hNoiseSpread = BASE_HORIZONTAL_NOISE + (settings.horizontalNoise * std::abs(deltaOdomHorizontal));
    horizontalDistribution = std::uniform_real_distribution<>(deltaOdomHorizontal - hNoiseSpread, 
                                                              deltaOdomHorizontal + hNoiseSpread);

    // 3. Angular noise remains centered around the current orientation
    angularDistribution = std::uniform_real_distribution<>(currentTheta - settings.angularNoise, 
                                                           currentTheta + settings.angularNoise);
}

void ParticleFilter::update_sensors() {
    for (auto&& sensor : sensors) {
        sensor->update();
    }
}

float ParticleFilter::weight_particles(float currentTheta) {
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

    return weightSum;
}

// std::pair<double, double> ParticleFilter::resample_particles(float weightSum) {
//     const double avgWeight = weightSum / static_cast<double>(NUM_PARTICLES);
//     std::uniform_real_distribution<float> distribution(0.0f, avgWeight);
//     const float randWeight = distribution(randomGenerator);

//     for (size_t i = 0; i < NUM_PARTICLES; i++) {
//         oldParticles[i] = particles[i];
//     }

//     size_t j = 0;

//     float cumulativeWeight = 0.0f;
//     float xSum = 0.0f, ySum = 0.0f;

//     for (size_t i = 0; i < NUM_PARTICLES; i++) {
//         const double weight = static_cast<double>(i) * avgWeight + randWeight;

//         while (cumulativeWeight < weight) {
//             if (j >= weights.size()) {
//                 break;
//             }
//             cumulativeWeight += weights[j];
//             j++;
//         }

//         particles[i].first = oldParticles[j-1].first;
//         particles[i].second = oldParticles[j-1].second;

//         xSum += particles[i].first;
//         ySum += particles[i].second;
//     }

//     return {xSum / static_cast<double>(NUM_PARTICLES), ySum / static_cast<double>(NUM_PARTICLES)};
// }

std::pair<double, double> ParticleFilter::resample_particles(float weightSum) {
    const double avgWeight = weightSum / static_cast<double>(NUM_PARTICLES);
    std::uniform_real_distribution<float> distribution(0.0f, avgWeight);
    const float randWeight = distribution(randomGenerator);

    oldParticles = particles; // Vector copy assignment is safe

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

    return totalWeight;
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

} // namespace cavalry
