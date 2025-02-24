#include "pid_controller.hpp"
#include <Arduino.h>

namespace m3508_control::pid_controller {
    /// @brief pid値のデバッグ出力の間隔(ループ回数)
    constexpr uint8_t DEBUG_PRINT_INTERVAL = 10;

    /// @brief PIDControllerクラスのコンストラクタ
    /// @param kp pゲイン
    /// @param ki iゲイン
    /// @param kd dゲイン
    /// @param clamping_output 最大出力(積分器のanti-windup用)
    PIDController::PIDController(
        const float kp, const float ki, const float kd, const float clamping_output,
        const bt_communication::BtInterface &bt_interface
    )
        : kp(kp),
          ki(ki),
          kd(kd),
          clamping_output(clamping_output),
          angle(0),
          rpm(0),
          amp(0),
          temp(0),
          target_rpm(0),
          integral(0),
          previous_error(0),
          bt_interface(bt_interface) {}

    /// @brief フィードバック値を設定
    /// @param angle モータの現在の角度(°)
    /// @param rpm モータの現在の回転数(rpm)
    /// @param amp モータに現在実際に流れている電流量(mA)
    /// @param temp モータの現在の温度(℃)
    void
    PIDController::set_feedback_values(const float angle, const int16_t rpm, const int16_t amp, const uint8_t temp) {
        static uint32_t count = 0;
        count++;
        this->angle = angle;
        this->rpm = rpm;
        this->amp = amp;
        this->temp = temp;
        if (count % DEBUG_PRINT_INTERVAL == 0) {
            Serial.print("Feedback set: \n");
            Serial.print(
                "angle: " + String(angle) + "deg, rpm: " + String(rpm) + "rpm, amp: " + String(amp) +
                "mA, temp: " + String(temp) + "deg C"
            );
            Serial.print("\n\n");
            bt_interface.remote_print("Feedback set: ");
            bt_interface.remote_print(
                "angle: " + String(angle) + "deg, rpm: " + String(rpm) + "rpm, amp: " + String(amp) +
                "mA, temp: " + String(temp) + "deg C"
            );
        }
    }

    /// @brief 制御目標値を設定
    /// @param target_rpm 制御目標値(rpm)
    void PIDController::set_target_rpm(const int16_t target_rpm) { this->target_rpm = target_rpm; }

    /// @brief 内部状態からPID出力値を計算し、内部状態を更新
    /// @return 出力値
    float PIDController::update_output() {
        static uint32_t count = 0;
        count++;

        const uint32_t dt = millis() - previous_update;
        const float current_error = static_cast<float>(target_rpm - rpm);

        // P値の計算
        const float proportional = kp * current_error;
        // I値の計算
        integral += ki * current_error * static_cast<float>(dt);
        // D値の計算
        const float derivative = kd * (current_error - previous_error) / static_cast<float>(dt);

        // 出力値の計算
        const float raw_output = proportional + integral + derivative;
        // 出力値を制限
        const float clamped_output = min(max(raw_output, -clamping_output), clamping_output);
        // 積分器のanti-windup
        if (raw_output != clamped_output && (raw_output * current_error > 0)) {
            integral = 0;
        }

        // ログ出力
        if (count % DEBUG_PRINT_INTERVAL == 0) {
            Serial.print("Output: \n");
            Serial.print(
                "output: " + String(clamped_output) + "mA, p: " + String(proportional) +
                ", i: " + String(integral) + ", d: " + String(derivative) + ", current rpm: " + String(rpm) +
                "rpm, target rpm: " + String(target_rpm) + "rpm, error: " + String(current_error) + "rpm"
            );
            Serial.print("\n\n");
            bt_interface.remote_print("Output: ");
            bt_interface.remote_print(
                "output: " + String(clamped_output) + "mA, p: " + String(proportional) +
                ", i: " + String(integral) + ", d: " + String(derivative) + ", current rpm: " + String(rpm) +
                "rpm, target rpm: " + String(target_rpm) + "rpm, error: " + String(current_error) + "rpm"
            );
        }
        bt_interface.remote_send_pid_fields(
            clamped_output, proportional, integral, derivative, target_rpm, current_error
        );

        previous_error = current_error;
        previous_update = millis();

        return clamped_output;
    }
}; // namespace m3508_control::pid_controller