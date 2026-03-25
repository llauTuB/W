#pragma once

#include "pros/distance.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/geom.hpp"

namespace cavalry {
constexpr auto DIST_WALL_FROM_ZERO = 72.0;
constexpr auto PARTICLE_UNCERTAINTY = 0.1f;
constexpr auto METERS_TO_INCHES = 39.3701;
constexpr auto INCHES_TO_METERS = 0.0254;

class DistanceSensor {
private:
    pros::Distance distance;
    lemlib::Pose offset;

    float measured = 0.0f;
    float standardDeviation = 0.0f;
    bool validReading = true;
    float tuningConstant = 0.0f;
    int objectSize = 0;
    bool disabled = false;
public:
    DistanceSensor(uint8_t port, const lemlib::Pose& offset, float tuningConstant) 
        : distance(std::move(pros::Distance(port))), offset(offset), tuningConstant(tuningConstant) {}
    void update();
    std::optional<double> p(const lemlib::Pose& particle, const std::vector<Circle>& circles, const std::vector<Polygon>& polygons);
    float get_distance() const {
        return measured;
    }
    int get_object_size() const {
        return objectSize;
    }
    bool is_valid_reading() const {
        return validReading && !disabled;
    }
    float get_standard_deviation() const {
        return standardDeviation;
    }
    float return_confidence()  {
        return distance.get_confidence();
    }
    void disable_sensor() {
        disabled = true;
    }
    void enable_sensor() {
        disabled = false;
    }
    bool is_disabled() const {
        return disabled;
    }
    ~DistanceSensor() = default;
private:
    float calculate_std_dev(float measured);
    float normal_pdf(const float x, const float mean, const float stddev);
    float fast_normal_pdf(const float x);
};
} // namespace cavalry