#pragma once

/// @brief 
///     2次元ベクトル
///
///     各種数値計算を提供します。
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

    /// @brief ベクトルを回転
    /// @param angle 回転角[deg]
    /// @return 回転されたベクトル
    Vec2 rotate(float angle) const;
};