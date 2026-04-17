#include "lemlib/distancesensor.hpp"

void daniyar::DistanceSensor::update() {
    const auto measuredMM = distance.get_distance();
    const auto measuredInches = static_cast<float>(measuredMM) / 1000.0f * METERS_TO_INCHES;

    objectSize = distance.get_object_size();
    validReading = measuredMM < 9999;
    measured = measuredInches * tuningConstant;
    standardDeviation = calculate_std_dev(measured);
}

std::optional<double> daniyar::DistanceSensor::p(const lemlib::Pose& particle, const std::vector<Circle>& circles, const std::vector<Polygon>& polygons) {
    if (!validReading) {
		return std::nullopt;
	}

    float angleGlobal = particle.theta + offset.theta;
    lemlib::Pose sensorGlobal = particle + offset.rotate(-particle.theta * (M_PI / 180.0f));

    // Convert angle to radians for direction vector
    // 0 is North (Y+), 90 is East (X+) -> Clockwise
    float angleRad = angleGlobal * (M_PI / 180.0f);
    Ray ray;
    ray.origin = {static_cast<float>(sensorGlobal.x), static_cast<float>(sensorGlobal.y)};
    ray.direction = {std::sin(angleRad), std::cos(angleRad)};

    float minDistance = std::numeric_limits<float>::infinity();

    // Check field walls (AABB intersection)
    if (std::abs(ray.direction.x) > 1e-6) {
        float t1 = (DIST_WALL_FROM_ZERO - ray.origin.x) / ray.direction.x;
        float t2 = (-DIST_WALL_FROM_ZERO - ray.origin.x) / ray.direction.x;
        if (t1 > 0 && t1 < minDistance) minDistance = t1;
        if (t2 > 0 && t2 < minDistance) minDistance = t2;
    }
    
    if (std::abs(ray.direction.y) > 1e-6) {
        float t1 = (DIST_WALL_FROM_ZERO - ray.origin.y) / ray.direction.y;
        float t2 = (-DIST_WALL_FROM_ZERO - ray.origin.y) / ray.direction.y;
        if (t1 > 0 && t1 < minDistance) minDistance = t1;
        if (t2 > 0 && t2 < minDistance) minDistance = t2;
    }

    // Check circles
    for (const auto& circle : circles) {
        float d = Geom::intersect(ray, circle);
        if (d < minDistance) minDistance = d;
    }

    // Check polygons
    for (const auto& poly : polygons) {
        float d = Geom::intersect(ray, poly);
        if (d < minDistance) minDistance = d;
    }

    double predicted = minDistance;
    if (std::isinf(predicted)) {
        predicted = 5000.0;
    }

    const float zScore = (measured - predicted) / standardDeviation;
    return normal_pdf(zScore, 0, 1);
}

float daniyar::DistanceSensor::calculate_std_dev(float measured) {
    if (measured < 8.0f) {
        return 0.05f * measured;
    }
    float confidence = distance.get_confidence();
    return 0.2f * measured * std::sqrt(confidence / 64.0f);
}

float daniyar::DistanceSensor::normal_pdf(const float x, const float mean, const float stddev) {
    return (1.0f / (stddev * std::sqrt(2.0f * M_PI))) * std::exp(-0.5f * ((x - mean) / stddev) * ((x - mean) / stddev));
}

float daniyar::DistanceSensor::fast_normal_pdf(const float x) {
    // Exit if z is too large for approximation
    if (std::abs(x) > 1.525f) {
        return PARTICLE_UNCERTAINTY;
    }

    const float coeff = 0.398942280401f; // 1 / sqrt(2*pi)
    const float x2 = x * x;
    const float x4 = x2 * x2;

    // Approximate normal PDF using Taylor Series
    return coeff * (1.0f - 0.5f * x2 + 0.125f * x4 - 0.020833f * x4 * x2);
}