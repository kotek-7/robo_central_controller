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
    ///     フィードバック値を受け取ってPID制御器にセットするためのread_and_set_feedback()関数、
    ///     シリアル通信を読み取ってPID制御器の目標値にセットするためのread_serial_and_set_target_rpm()関数を提供します。
    ///
    ///     このクラスは、M3508モータの制御を行うために、内部にPID制御器(PIDControllerオブジェクト)を持ちます。
    ///     また、電流値を送信するためのCanTransmitterオブジェクトと、
    ///     Bluetoothでモニタに様々な情報を送るためのBtInterfaceオブジェクトも持ちます。
    ///     M3508からのFBをCANで受信する処理は、外部で行われて、受信結果だけ渡されることを想定しています。
    /// @see 
    ///     https://www.mouser.com/datasheet/2/744/RoboMaster_M3508_P19_Brushless_DC_Gear_Motor_V1.0-1551061.pdf
    ///     https://www.mouser.com/datasheet/2/744/RoboMaster_C620_Brushless_DC_Motor_Speed_Controlle-1551090.pdf
    class M3508Controller {
    public:
        M3508Controller(
            const bt_communication::BtJsonSender &bt_json_sender,
            const bt_communication::BtPrinter &bt_printer,
            const can::CanTransmitter &can_transmitter
        );

        /// @brief PIDで電流値を計算してM3508にCANで送信
        void send_currents();
        /// @brief CANで受信したM3508のフィードバック値をPID制御器にセット (CAN通信の読み取りは行わない)
        /// @param rx_id CANで受信したメッセージのID
        /// @param rx_buf CANで受信したメッセージのデータ
        void set_feedback(const can::CanId rx_id, const std::array<uint8_t, 8> rx_buf);
        /// @brief シリアル通信を読み取ってPIDの目標値を設定
        void read_serial_and_set_target_rpm();

        /// @brief PID制御器のpゲインを設定する
        void set_kp(float p);
        /// @brief PID制御器のiゲインを設定する
        void set_ki(float i);
        /// @brief PID制御器のdゲインを設定する
        void set_kd(float d);
        /// @brief PID制御器の目標値(rpm)を設定する
        /// @param target_rpm 目標rpm [rpm]
        void set_target_rpm(float target_rpm);

        /// @brief 機体の目標速度を設定する
        /// @param target_velocity 機体の目標速度ベクトル [m/s]
        void set_target_velocity(const Vec2 &target_velocity);
        /// @brief 機体の目標角速度を設定する
        /// @param target_angular_velocity 機体の目標角速度 [deg/s]
        void set_target_angular_velocity(const float target_angular_velocity);

        /// @brief センサから読み取られた機体の現在のyaw角度を知らせる
        /// @param yaw 機体のyaw角度 [deg]
        void set_yaw(float yaw);
        /// @brief センサから読み取られた機体の現在のyaw角速度を知らせる
        /// @param yaw_velocity 機体のyaw角速度 [deg/s]
        void set_yaw_velocity(float yaw_velocity);

    private:
        /// @brief PID制御器(制御の核！)
        std::unordered_map<C620Id, m3508_control::pid_controller::PIDController, C620IdHash> pid_controllers;

        /// @brief 機体の目標速度[m/s]
        Vec2 target_velocity;
        /// @brief 機体の目標角速度[deg/s]
        float target_angular_velocity;

        /// @brief センサから読み取られた機体の現在のyaw角度[deg]
        float yaw;

        /// @brief センサから読み取られた機体の現在のyaw角速度[deg/s]
        float yaw_velocity;

        /// @brief 送信する電流値(mA)のバッファ
        int32_t command_currents[4];

        /// @brief 汎用Bluetooth通信用インスタンス
        const bt_communication::BtJsonSender &bt_json_sender;
        /// @brief Bluetoothモニタにログを送るためのインスタンス
        const bt_communication::BtPrinter &bt_printer;

        /// @brief CAN送信用のインスタンス
        const can::CanTransmitter &can_transmitter;

        /// @brief BluetoothモニタにM3508のフィードバック値を送信
        void remote_send_feedback(
            C620Id c620_id, float angle, int16_t rpm, int16_t amp, uint8_t temp
        ) const;

        /// @brief 4つの電流値を、CANで速度コントローラに送信するデータへ変換
        /// @param milli_amperes
        ///     4つの-20000\~20000の電流値(mA)を格納した配列
        ///     (要素番号と速度コントローラIDが対応)
        /// @param out_tx_buf 結果の書き込み用配列
        void milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]);

        /// @brief 速度コントローラから受け取ったデータから、フィードバック値を導出
        /// @param rx_buf CANで受信した配列
        /// @param out_angle ロータの角度(0°\~360°) (結果書き込み用)
        /// @param out_rpm 回転速度(rpm) (結果書き込み)
        /// @param out_amp 実際のトルク電流(mA) (結果書き込み用)
        /// @param out_temp モータの温度(℃) (結果書き込み用)
        ///
        /// TODO: std::arrayを受け取るようにする
        void derive_feedback_fields(
            const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp
        );

        /// @brief 機体の目標速度と目標角速度から、各モータの目標rpmを計算
        /// @param target_velocity 機体の目標速度 [m/s]
        /// @param target_angular_velocity 機体の目標角速度 [deg/s]
        /// @param current_yaw 機体の現在のyaw角度 [deg]
        /// @param out_target_rpm_1 モータ1の目標rpm (結果書き込み用)
        /// @param out_target_rpm_2 モータ2の目標rpm (結果書き込み用)
        /// @param out_target_rpm_3 モータ3の目標rpm (結果書き込み用)
        /// @param out_target_rpm_4 モータ4の目標rpm (結果書き込み用)
        void calc_target_rpms(
            const Vec2 &target_velocity,
            const float target_angular_velocity,
            const float current_yaw,
            float *out_target_rpm_1,
            float *out_target_rpm_2,
            float *out_target_rpm_3,
            float *out_target_rpm_4
        );
    };
} // namespace m3508_control