#pragma once

#include "bt_communication/bt_interface.hpp"
#include <Arduino.h>

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
            const float kp, const float ki, const float kd, const float clamping_output,
            const bt_communication::BtInterface &bt_interface
        );

        /// @brief kpを設定する
        void set_kp(const float kp) { this->kp = kp; };
        /// @brief kiを設定する
        void set_ki(const float ki) { this->ki = ki; };
        /// @brief kdを設定する
        void set_kd(const float kd) { this->kd = kd; };

        void set_feedback_values(const float angle, const int16_t rpm, const int16_t amp, const uint8_t temp);
        void set_target_rpm(const int16_t target_rpm);
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

        /// @brief Bluetooth通信用の関数集めたやつ
        const bt_communication::BtInterface &bt_interface;
    };

} // namespace m3508_control::pid_controller
