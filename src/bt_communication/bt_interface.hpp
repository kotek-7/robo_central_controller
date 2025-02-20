#pragma once
#include <Arduino.h>

namespace bt_interface {
    class BtInterface {
    public:
        BtInterface(
            std::function<void(String)> remote_print,
            std::function<void(float angle, int16_t rpm, int16_t amp, uint8_t temp)> remote_send_feedback,
            std::function<void(float output, float p, float i, float d, float target_rpm, float error)>
                remote_send_pid_fields
        )
            : remote_print(remote_print),
              remote_send_feedback(remote_send_feedback),
              remote_send_pid_fields(remote_send_pid_fields) {};
        std::function<void(String)> remote_print;
        std::function<void(float angle, int16_t rpm, int16_t amp, uint8_t temp)> remote_send_feedback;
        std::function<void(float output, float p, float i, float d, float target_rpm, float error)>
            remote_send_pid_fields;
    };
} // namespace bt_interface