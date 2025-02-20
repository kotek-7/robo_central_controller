#pragma once

#include "pid_controller/pid_controller.hpp"
#include <Arduino.h>

namespace m3508_controller {
    class M3508Controller {
    public:
        M3508Controller(const bt_communication::BtInterface &bt_interface);
        void setup();
        void loop();

    private:
        /// @brief PID制御器(制御の核！)
        m3508_controller::pid_controller::PIDController pid_controller;

        /// @brief 前回のCAN送信時間
        uint32_t previous_can_send_millis;
        /// @brief 前回のCAN受信時間
        uint32_t previous_can_receive_millis;
        /// @brief 前回のシリアル受信時間
        uint32_t previous_serial_read_millis;

        const bt_communication::BtInterface &bt_interface;

        void milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]);
        void derive_feedback_fields(
            const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp
        );
    };
} // namespace m3508_controller