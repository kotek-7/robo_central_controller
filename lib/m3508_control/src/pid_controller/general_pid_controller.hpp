#pragma once

#include <bt_communication/peripheral.hpp>

namespace m3508_control::pid_controller {
    class GeneralPIDController {
    public:
        GeneralPIDController(
            const float kp,
            const float ki,
            const float kd,
            const float clamping_output,
            const bt_communication::BtPrinter &bt_printer
        );

        void set_kp(const float kp) { this->kp = kp; };
        void set_ki(const float ki) { this->ki = ki; };
        void set_kd(const float kd) { this->kd = kd; };
        void set_feedback_value(const float feedback_value) { this->feedback_value = feedback_value; };
        void set_target_value(const float target_value) {};

        float update_output();
    private:
        float kp;
        float ki;
        float kd;
        const float clamping_output;

        float target_value;
        float feedback_value;
        float integral;
        float previous_error;
        uint32_t previous_update_millis;
        uint32_t update_count;
        const bt_communication::BtPrinter &bt_printer;

    };
} // namespace m3508control::pid_controller
