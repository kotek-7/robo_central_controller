#include <Arduino.h>
#include "general_pid_controller.hpp"

namespace m3508_control::pid_controller
{
    GeneralPIDController::GeneralPIDController(
        const float kp,
        const float ki,
        const float kd,
        const float clamping_output,
        const bt_communication::BtPrinter &bt_printer
    ) :
        kp(kp),
        ki(ki),
        kd(kd),
        clamping_output(clamping_output),
        target_value(0),
        feedback_value(0),
        integral(0),
        previous_error(0),
        previous_update_millis(millis()),
        update_count(0),
        bt_printer(bt_printer) {}

    float GeneralPIDController::update_output() {
        update_count++;

        const float dt = static_cast<float>(millis() - previous_update_millis) / 1000.0f;
        previous_update_millis = millis();

        const float error = target_value - feedback_value;

        float proportional = kp * error;
        integral += ki * error * dt;
        float derivative = kd * (error - previous_error) / dt;
        previous_error = error;

        float raw_output = proportional + integral + derivative;
        float clamped_output = std::clamp(raw_output, -clamping_output, clamping_output);
        if (raw_output != clamped_output && (raw_output * error > 0)) {
            integral = 0;
        }

        return clamped_output;
    }
    
} // namespace m3508control::pid_controller
