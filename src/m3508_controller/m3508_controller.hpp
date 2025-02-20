#pragma once

#include "pid_controller/pid_controller.hpp"
#include <Arduino.h>

namespace m3508_controller {
    /// @brief M3508モータの制御を行うクラス
    /// @details
    ///     M3508モータの制御を行うクラスです。
    ///     このクラスは、M3508モータの制御を行うために、内部にPID制御器(PIDControllerオブジェクト)を持ちます。
    ///     また、Bluetoothでモニタに情報を送るためのBtInterfaceオブジェクトも持ちます。
    ///     main.cpp内でsetup()関数を呼び出すことで初期化を行い、loop()関数を呼び出すことで制御を行います。
    ///     M3508モータの角度、回転数、電流、温度を受信し、pid_controllerに渡します。
    ///     また、pid_controllerから出力された値をM3508モータに送信します。
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