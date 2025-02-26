#pragma once

#include "m3508_control/m3508_controller.hpp"
#include "m3508_control/c620_id.hpp"
#include <Arduino.h>

namespace bt_communication {
    // Bluetoothでいろいろするための関数集
    class BtInterface {
    public:
        BtInterface(
            std::function<void(String)> remote_print,
            std::function<void(m3508_control::C620Id c620_id, float angle, int16_t rpm, int16_t amp, uint8_t temp)>
                remote_send_feedback,
            std::function<void(m3508_control::C620Id c620_id, float output, float p, float i, float d, float target_rpm, float error)>
                remote_send_pid_fields
        )
            : remote_print(remote_print),
              remote_send_feedback(remote_send_feedback),
              remote_send_pid_fields(remote_send_pid_fields) {};
        /// @brief モニターのコンソールに文字列を表示する
        std::function<void(String)> remote_print;
        /// @brief モニターにM3508のフィードバックを送信する
        std::function<void(m3508_control::C620Id c620_id, float angle, int16_t rpm, int16_t amp, uint8_t temp)> remote_send_feedback;
        /// @brief モニターにPIDのパラメータを送信する
        std::function<void(m3508_control::C620Id c620_id, float output, float p, float i, float d, float target_rpm, float error)>
            remote_send_pid_fields;
    };
} // namespace bt_communication