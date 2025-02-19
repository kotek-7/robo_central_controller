#pragma once

#include <Arduino.h>

namespace m3508_controller::pid_controller {
    /// @brief PID制御を行う。フィードバック値を保持し、出力値を計算する。
    /// @details
    /// update_output()を定期的に呼び出すことで、出力値をPID制御の計算によって取得しつつ、内部の積分器などの変数を更新していく。
    /// フィードバック値を受け取るたびにset_feedback_values()を呼び出すことで、内部のフィードバック値を更新できる。
    /// 制御目標値を変更する場合は、set_target_rpm()を呼び出す。
    class PIDController {
    public:
        PIDController(
            const float kp, const float ki, const float kd, const float clamping_output, const uint32_t interval,
            std::function<void(String)> remote_print,
            std::function<void(float output, float p, float i, float d, float target_rpm, float error)> remote_send_pid_fields
        );

        void set_feedback_values(const float angle, const int16_t rpm, const int16_t amp, const uint8_t temp);

        void set_target_rpm(const int16_t target_rpm);

        float update_output();

    private:
        /// @brief PID制御のpゲイン
        const float kp;
        /// @brief PID制御のiゲイン
        const float ki;
        /// @brief PID制御のdゲイン
        const float kd;
        /// @brief 最大出力(積分器のanti-windup用)
        const float clamping_output;
        /// @brief update_output()の実行間隔
        const uint32_t interval;

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
        float previous_error;

        /// @brief モニタにPID制御値を送信
        std::function<void(float output, float p, float i, float d, float target_rpm, float error)> remote_send_pid_fields;
        /// @brief モニタのコンソールにテキストを送信
        std::function<void(String)> remote_print;
    };

} // namespace m3508_controller::pid_controller
