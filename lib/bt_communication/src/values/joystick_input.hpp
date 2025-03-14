#pragma once

#include <vec2.hpp>

namespace bt_communication::joystick_input {

    /// @brief ジョイスティックの入力を保持する。
    class JoystickInput {
    public:
        JoystickInput(Vec2 input, Vec2 leveled_input, float distance, float angle);
        JoystickInput();

        const Vec2 *get_input() const { return &input; };
        const Vec2 *get_leveled_input() const { return &leveled_input; };
        float get_distance() const { return distance; };
        float get_angle() const { return angle; };

    private:
        /// @brief ジョイスティックの入力値
        Vec2 input;

        /// @brief ジョイスティックの入力値を正規化した値(-1~1)
        Vec2 leveled_input;

        /// @brief ジョイスティックの入力値の極座標表現
        float distance;
        float angle;
    };

} // namespace bt_communication::joystick_input