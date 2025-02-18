#pragma once

namespace utils {
    /// @brief 2次元ベクトル
    class Vec2 {
    public:
        float x;
        float y;

        Vec2(float x, float y);

        Vec2 operator+(const Vec2 &v) const;
        Vec2 operator-(const Vec2 &v) const;
        Vec2 operator*(const float &s) const;
        Vec2 operator/(const float &s) const;

        float dot(const Vec2 &v) const;
        float cross(const Vec2 &v) const;
        float length() const;
        Vec2 normalized() const;
    };
} // namespace utils