#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

namespace rwb {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double x_value, double y_value) : x(x_value), y(y_value) {}

    Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
    Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
    Vec2 operator*(double scale) const { return {x * scale, y * scale}; }
    Vec2 operator/(double scale) const { return {x / scale, y / scale}; }
    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    double dot(const Vec2& other) const { return x * other.x + y * other.y; }
    double norm() const { return std::sqrt(dot(*this)); }
};

inline double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

inline double clamp01(double value) { return clamp(value, 0.0, 1.0); }

inline double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

inline double smoothstep(double t) {
    t = clamp01(t);
    return t * t * (3.0 - 2.0 * t);
}

inline double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

inline double logistic(double value) {
    if (value >= 0.0) {
        const double exponential = std::exp(-value);
        return 1.0 / (1.0 + exponential);
    }
    const double exponential = std::exp(value);
    return exponential / (1.0 + exponential);
}

inline double sq(double value) { return value * value; }

struct Rect {
    double min_x;
    double min_y;
    double max_x;
    double max_y;
};

inline double point_rect_distance(const Vec2& point, const Rect& rect) {
    const double dx = std::max({rect.min_x - point.x, 0.0, point.x - rect.max_x});
    const double dy = std::max({rect.min_y - point.y, 0.0, point.y - rect.max_y});
    if (dx > 0.0 || dy > 0.0) {
        return std::sqrt(dx * dx + dy * dy);
    }
    const double inside_x = std::min(point.x - rect.min_x, rect.max_x - point.x);
    const double inside_y = std::min(point.y - rect.min_y, rect.max_y - point.y);
    return -std::min(inside_x, inside_y);
}

class Rng {
public:
    explicit Rng(std::uint64_t seed) : engine_(seed) {}

    double uniform(double low, double high) {
        return std::uniform_real_distribution<double>(low, high)(engine_);
    }

    int uniform_int(int low, int high) {
        return std::uniform_int_distribution<int>(low, high)(engine_);
    }

    bool chance(double probability) {
        return uniform(0.0, 1.0) < probability;
    }

    std::uint64_t raw() { return engine_(); }

private:
    std::mt19937_64 engine_;
};

}  // namespace rwb

