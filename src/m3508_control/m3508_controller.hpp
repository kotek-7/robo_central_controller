#pragma once

#include <unordered_map>
#include "c620_id.hpp"
#include "pid_controller/pid_controller.hpp"

namespace m3508_control {
    using CanId = uint32_t;

    /// @brief M3508モータの制御を行うクラス
    /// @details
    ///     M3508モータの制御を行うクラスです。
    ///
    ///     電流値をCANで送信するためのsend_currents()関数、
    ///     CANでフィードバック値を読み取ってPID制御器にセットするためのread_and_set_feedback()関数、
    ///     シリアル通信を読み取ってPID制御器の目標値にセットするためのread_serial_and_set_target_rpm()関数を提供します。
    ///
    ///     使う前にsetup()関数を呼び出して初期化する必要があります。
    ///     このクラスは、M3508モータの制御を行うために、内部にPID制御器(PIDControllerオブジェクト)を持ちます。
    ///     また、Bluetoothでモニタに様々な情報を送るためのBtInterfaceオブジェクトも持ちます。
    class M3508Controller {
    public:
        M3508Controller(const bt_communication::BtInterface &bt_interface);
        void setup();
        void send_currents();
        void read_and_set_feedback();
        void read_serial_and_set_target_rpm();

        /// @brief PID制御器のpゲインを設定する
        void set_kp(float p) {
            pid_controller.set_kp(p);
        };
        /// @brief PID制御器のiゲインを設定する
        void set_ki(float i) {
            pid_controller.set_ki(i);
        };
        /// @brief PID制御器のdゲインを設定する
        void set_kd(float d) {
            pid_controller.set_kd(d);
        };
        /// @brief PID制御器の目標値(rpm)を設定する
        void set_target_rpm(float target_rpm) {
            pid_controller.set_target_rpm(target_rpm);
        };

    private:
        /// @brief PID制御器(制御の核！)
        m3508_control::pid_controller::PIDController pid_controller;

        /// @brief 送信する電流値(mA)のバッファ
        int32_t command_currents[4];

        const bt_communication::BtInterface &bt_interface;

        void milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]);
        void derive_feedback_fields(
            const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp
        );
    };
} // namespace m3508_control