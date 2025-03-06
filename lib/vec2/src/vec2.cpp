#include <cmath>
#include "vec2.hpp"

Vec2::Vec2(float x, float y) :
    x(x), y(y) {}

Vec2 Vec2::operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }

Vec2 Vec2::operator-(const Vec2 &v) const { return Vec2(x - v.x, y - v.y); }

Vec2 Vec2::operator*(const float &s) const { return Vec2(x * s, y * s); }

Vec2 Vec2::operator/(const float &s) const { return Vec2(x / s, y / s); }

float Vec2::dot(const Vec2 &v) const { return x * v.x + y * v.y; }

float Vec2::cross(const Vec2 &v) const { return x * v.y - y * v.x; }

float Vec2::length() const { return std::sqrt(x * x + y * y); }

Vec2 Vec2::normalized() const { return *this / length(); }

Vec2 Vec2::rotate(float angle) const {
    float rad = angle * M_PI / 180.0;
    float cos_rad = std::cos(rad);
    float sin_rad = std::sin(rad);
    return Vec2(x * cos_rad - y * sin_rad, x * sin_rad + y * cos_rad);
}