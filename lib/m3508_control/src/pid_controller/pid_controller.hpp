#pragma once

#include <Arduino.h>
#include <bt_communication/peripheral.hpp>

namespace m3508_control::pid_controller {
    /// @brief PID制御を行う。フィードバック値を保持し、出力値を計算する。
    /// @details
    ///     このクラスはM3508モータのPID制御を行うためのクラスです。フィードバック値を受取り、出力値を計算します。
    ///     (デバッグ以外の通信は行いません。計算機です。)
    ///     update_output()を定期的に呼び出すことで、出力値を計算して取得したあと、内部の積分器などの変数を更新できます。
    ///     フィードバック値を受け取るたびにset_feedback_values()を呼び出して、内部のフィードバック値を更新します。
    ///     制御目標値を変更する場合は、set_target_rpm()を呼び出します。
    ///     また、Bluetooth通信用のBtInterfaceオブジェクトも持っており、それを通じてモニターに情報を送信します。
    class PIDController {
    public:
        PIDController(
            const float kp,
            const float ki,
            const float kd,
            const float clamping_output,
            const bt_communication::BtPrinter &bt_printer,
            const std::function<void(float output, float proportional, float integral, float derivative, float target_rpm, float error)>
                remote_send_pid_fields
        );

        /// @brief kpを設定する
        void set_kp(const float kp) { this->kp = kp; };
        /// @brief kiを設定する
        void set_ki(const float ki) { this->ki = ki; };
        /// @brief kdを設定する
        void set_kd(const float kd) { this->kd = kd; };
        /// @brief フィードバック値を設定
        /// @param angle モータの現在の角度(°)
        /// @param rpm モータの現在の回転数(rpm)
        /// @param amp モータに現在実際に流れている電流量(mA)
        /// @param temp モータの現在の温度(℃)
        void set_feedback_values(const float angle, const int16_t rpm, const int16_t amp, const uint8_t temp);
        /// @brief 制御目標値を設定
        /// @param target_rpm 制御目標値(rpm)
        void set_target_rpm(const int16_t target_rpm);

        /// @brief 内部状態からPID出力値を計算し、内部状態を更新
        /// @return 出力値(mA)
        float update_output();

    private:
        /// @brief PID制御のpゲイン
        float kp;
        /// @brief PID制御のiゲイン
        float ki;
        /// @brief PID制御のdゲイン
        float kd;
        /// @brief 最大出力(積分器のanti-windup用)
        const float clamping_output;

        /// @brief モータの現在の角度(°)
        float angle;
        /// @brief モータの現在の回転数(rpm)
        int16_t rpm;
        /// @brief モータに現在実際に流れている電流量(mA)
        int16_t amp;
        /// @brief モータの現在の温度(℃)
        int8_t temp;

        /// @brief 制御目標値(rpm)
        int16_t target_rpm;

        float integral;
        uint32_t previous_update;
        float previous_error;

        const bt_communication::BtPrinter &bt_printer;
        std::function<void(float output, float proportional, float integral, float derivative, float target_rpm, float error)>
            remote_send_pid_fields;
    };

} // namespace m3508_control::pid_controller
