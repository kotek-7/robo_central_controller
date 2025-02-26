#include <cmath>
#include "vec2.hpp"

namespace utils {
    Vec2::Vec2(float x, float y) : x(x), y(y) {}

    Vec2 Vec2::operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }

    Vec2 Vec2::operator-(const Vec2 &v) const { return Vec2(x - v.x, y - v.y); }

    Vec2 Vec2::operator*(const float &s) const { return Vec2(x * s, y * s); }

    Vec2 Vec2::operator/(const float &s) const { return Vec2(x / s, y / s); }

    float Vec2::dot(const Vec2 &v) const { return x * v.x + y * v.y; }

    float Vec2::cross(const Vec2 &v) const { return x * v.y - y * v.x; }

    float Vec2::length() const { return std::sqrt(x * x + y * y); }

    Vec2 Vec2::normalized() const { return *this / length(); }

} // namespace utils