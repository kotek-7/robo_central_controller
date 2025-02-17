#pragma once

#include <Arduino.h>

namespace m3508_controller::pid_controller
{

    class PIDController
    {
    private:
        const float kp;
        const float ki;
        const float kd;
        const float clamping_output;
        const uint32_t interval;

        float angle;
        int16_t rpm;
        int16_t amp;
        int8_t temp;

        int16_t target_rpm;

        float integral;
        float previous_error;
        uint32_t count;

    public:
        PIDController(const float kp, const float ki, const float kd, const float clamping_output, const uint32_t interval);

        void set_feedback_values(const float angle, const int16_t rpm, const int16_t amp, const uint8_t temp);

        void set_target_rpm(const int16_t target_rpm);

        float update_output();
    };

}
