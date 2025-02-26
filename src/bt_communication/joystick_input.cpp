#include "joystick_input.hpp"
#include "utils/vec2.hpp"
#include <Arduino.h>

namespace bt_communication::joystick_input {
    JoystickInput::JoystickInput()
        : input(utils::Vec2(0, 0)), leveled_input(utils::Vec2(0, 0)), distance(0), angle(0) {}

    JoystickInput::JoystickInput(utils::Vec2 input, utils::Vec2 leveled_input, float distance, float angle)
        : input(input), leveled_input(leveled_input), distance(distance), angle(angle) {}
} // namespace bt_communication::joystick_input