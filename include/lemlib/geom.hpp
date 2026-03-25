#pragma once
#include <vector>
#include <cmath>
#include <limits>

namespace cavalry{

struct Point {
    float x, y;
};

struct Ray {
    Point origin;
    Point direction; // Normalizedz
};

struct Circle {
    Point center;
    float radius;
};

struct Polygon {
    std::vector<Point> vertices;
};

namespace Geom {

    inline float distSq(Point p1, Point p2) {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
    }

    // Ray-Circle Intersection
    // Returns distance to intersection, or infinity if no intersection
    inline float intersect(const Ray& ray, const Circle& circle) {
        float ox = ray.origin.x - circle.center.x;
        float oy = ray.origin.y - circle.center.y;

        float a = ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y; // Should be 1.0 if normalized
        float b = 2.0f * (ox * ray.direction.x + oy * ray.direction.y);
        float c = ox * ox + oy * oy - circle.radius * circle.radius;

        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0) {
            return std::numeric_limits<float>::infinity();
        } else {
            float t1 = (-b - std::sqrt(discriminant)) / (2.0f * a);
            float t2 = (-b + std::sqrt(discriminant)) / (2.0f * a);

            if (t1 > 0) return t1;
            if (t2 > 0) return t2;
            return std::numeric_limits<float>::infinity();
        }
    }

    // Ray-LineSegment Intersection
    // Returns distance to intersection, or infinity
    inline float intersectSegment(const Ray& ray, Point p1, Point p2) {
        float v1x = ray.origin.x - p1.x;
        float v1y = ray.origin.y - p1.y;
        float v2x = p2.x - p1.x;
        float v2y = p2.y - p1.y;
        float v3x = -ray.direction.x;
        float v3y = -ray.direction.y;

        float dot = v2x * v3y - v2y * v3x;
        if (std::abs(dot) < 0.000001f) return std::numeric_limits<float>::infinity();

        float t1 = (v2x * v1y - v2y * v1x) / dot;
        float t2 = (v3x * v1y - v3y * v1x) / dot;

        if (t1 >= 0.0f && (t2 >= 0.0f && t2 <= 1.0f)) {
            return t1;
        }
        return std::numeric_limits<float>::infinity();
    }

    // Ray-Polygon Intersection
    inline float intersect(const Ray& ray, const Polygon& poly) {
        float minDist = std::numeric_limits<float>::infinity();
        size_t n = poly.vertices.size();
        for (size_t i = 0; i < n; ++i) {
            float d = intersectSegment(ray, poly.vertices[i], poly.vertices[(i + 1) % n]);
            if (d < minDist) {
                minDist = d;
            }
        }
        return minDist;
    }
}

} // namespace cavalry