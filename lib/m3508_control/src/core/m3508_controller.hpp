#pragma once

#include <unordered_map>
#include <can/peripheral.hpp>
#include <vec2.hpp>
#include "../values/c620_id.hpp"
#include "../pid_controller/pid_controller.hpp"

namespace m3508_control {
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
        M3508Controller(const bt_communication::BtInterface &bt_interface, const can::CanTransmitter &can_transmitter);
        void send_currents();
        void set_feedback(const can::CanId rx_id, const std::array<uint8_t, 8> rx_buf);
        void read_serial_and_set_target_rpm();

        /// @brief PID制御器のpゲインを設定する
        void set_kp(float p) {
            pid_controllers.at(C620Id::C1).set_kp(p);
            pid_controllers.at(C620Id::C2).set_kp(p);
            pid_controllers.at(C620Id::C3).set_kp(p);
            pid_controllers.at(C620Id::C4).set_kp(p);
        };
        /// @brief PID制御器のiゲインを設定する
        void set_ki(float i) {
            pid_controllers.at(C620Id::C1).set_ki(i);
            pid_controllers.at(C620Id::C2).set_ki(i);
            pid_controllers.at(C620Id::C3).set_ki(i);
            pid_controllers.at(C620Id::C4).set_ki(i);
        };
        /// @brief PID制御器のdゲインを設定する
        void set_kd(float d) {
            pid_controllers.at(C620Id::C1).set_kd(d);
            pid_controllers.at(C620Id::C2).set_kd(d);
            pid_controllers.at(C620Id::C3).set_kd(d);
            pid_controllers.at(C620Id::C4).set_kd(d);
        };
        /// @brief PID制御器の目標値(rpm)を設定する
        void set_target_rpm(float target_rpm) {
            pid_controllers.at(C620Id::C1).set_target_rpm(target_rpm);
            pid_controllers.at(C620Id::C2).set_target_rpm(target_rpm);
            pid_controllers.at(C620Id::C3).set_target_rpm(target_rpm);
            pid_controllers.at(C620Id::C4).set_target_rpm(target_rpm);
        };

        void set_target_velocity(const Vec2 &target_velocity);
        void set_target_angular_velocity(const float target_angular_velocity);

    private:
        /// @brief PID制御器(制御の核！)
        std::unordered_map<C620Id, m3508_control::pid_controller::PIDController, C620IdHash> pid_controllers;
        Vec2 target_velocity;
        float target_angular_velocity;
        /// @brief 送信する電流値(mA)のバッファ
        int32_t command_currents[4];

        const bt_communication::BtInterface &bt_interface;
        const can::CanTransmitter &can_transmitter;

        void milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]);
        void derive_feedback_fields(
            const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp
        );
        void calc_target_rpms(
            const Vec2 &target_velocity,
            const float target_angular_velocity,
            float *out_target_rpm_1,
            float *out_target_rpm_2,
            float *out_target_rpm_3,
            float *out_target_rpm_4
        );
    };
} // namespace m3508_control