#include "pid_controller.hpp"
#include <Arduino.h>

namespace m3508_controller::pid_controller {
    /// @brief pid値のデバッグ出力の間隔(ループ回数)
    constexpr uint8_t DEBUG_PRINT_INTERVAL = 10;

    /// @brief PIDControllerクラスのコンストラクタ
    /// @param kp pゲイン
    /// @param ki iゲイン
    /// @param kd dゲイン
    /// @param clamping_output 最大出力(積分器のanti-windup用)
    /// @param interval update_output()の実行間隔(dtとして計算に使うため)
    PIDController::PIDController(
        const float kp, const float ki, const float kd, const float clamping_output, const uint32_t interval,
        std::function<void(String)> remote_print,
        std::function<void(float output, float p, float i, float d, float target_rpm, float error)> remote_send_pid_fields
    )
        : kp(kp),
          ki(ki),
          kd(kd),
          clamping_output(clamping_output),
          interval(interval),
          remote_print(remote_print),
          remote_send_pid_fields(remote_send_pid_fields),
          integral(0),
          previous_error(0),
          target_rpm(0),
          angle(0),
          rpm(0),
          amp(0),
          temp(0) {}

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
            Serial.println("Received: ");
            Serial.print("angle: " + String(angle) + "°,");
            Serial.print("rpm: " + String(rpm) + "rpm, ");
            Serial.print("amp: " + String(amp) + "mA, ");
            Serial.print("temp: " + String(temp) + "℃");
            Serial.print("\n\n");
            remote_print("Received: ");
            remote_print(
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

        float current_error = static_cast<float>(target_rpm - rpm);
        integral += current_error * static_cast<float>(interval);
        float derivative = (current_error - previous_error) / static_cast<float>(interval);
        float raw_output = kp * current_error + ki * integral + kd * derivative;

        float clamped_output = min(max(raw_output, -clamping_output), clamping_output);
        if (raw_output != clamped_output && (raw_output * current_error > 0)) {
            integral = 0;
        }

        if (count % DEBUG_PRINT_INTERVAL == 0) {
            Serial.print("Sent: \n");
            Serial.print("output: " + String(clamped_output) + "mA, ");
            Serial.print("p: " + String(kp * current_error) + ", ");
            Serial.print("i: " + String(ki * integral) + ", ");
            Serial.print("d: " + String(kd * derivative) + ", ");
            Serial.print("current rpm: " + String(rpm) + "rpm, ");
            Serial.print("target rpm: " + String(target_rpm) + "rpm, ");
            Serial.print("error: " + String(current_error) + "rpm, ");
            Serial.print("\n\n");
            remote_print("Sent: ");
            remote_print(
                "output: " + String(clamped_output) + "mA, p: " + String(kp * current_error) +
                ", i: " + String(ki * integral) + ", d: " + String(kd * derivative) + ", current rpm: " + String(rpm) +
                "rpm, target rpm: " + String(target_rpm) + "rpm, error: " + String(current_error) + "rpm"
            );
        }
        remote_send_pid_fields(clamped_output, kp * current_error, ki * integral, kd * derivative, target_rpm, current_error);

        previous_error = current_error;

        return clamped_output;
    }
}; // namespace m3508_controller::pid_controller