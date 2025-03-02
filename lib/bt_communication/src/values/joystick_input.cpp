#include <Arduino.h>
#include "joystick_input.hpp"

namespace bt_communication::joystick_input {
    JoystickInput::JoystickInput()
        : input(Vec2(0, 0)), leveled_input(Vec2(0, 0)), distance(0), angle(0) {}

    JoystickInput::JoystickInput(Vec2 input, Vec2 leveled_input, float distance, float angle)
        : input(input), leveled_input(leveled_input), distance(distance), angle(angle) {}
} // namespace bt_communication::joystick_input