#include "pid_controller.hpp"

namespace m3508_control::pid_controller {
    /// @brief pid値のデバッグ出力の間隔(ループ回数)
    constexpr uint8_t DEBUG_PRINT_INTERVAL = 10;

    /// @brief PIDControllerクラスのコンストラクタ
    /// @param kp pゲイン
    /// @param ki iゲイン
    /// @param kd dゲイン
    /// @param clamping_output 最大出力(積分器のanti-windup用)
    PIDController::PIDController(
        const float kp,
        const float ki,
        const float kd,
        const float clamping_output,
        std::function<void(String)> remote_print,
        std::function<
            void(float output, float proportional, float integral, float derivative, float target_rpm, float error)>
            remote_send_pid_fields
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
          previous_update(millis()),
          previous_error(0),
          remote_print(remote_print),
          remote_send_pid_fields(remote_send_pid_fields) {}

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
            remote_print("Feedback set: ");
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
    /// @return 出力値(mA)
    float PIDController::update_output() {
        static uint32_t count = 0;
        count++;

        const uint32_t dt = millis() - previous_update;
        const float error = static_cast<float>(target_rpm - rpm);

        // P値の計算
        const float proportional = kp * error;
        // I値の計算
        integral += ki * error * static_cast<float>(dt);
        // D値の計算
        const float derivative = kd * (error - previous_error) / static_cast<float>(dt);

        // 出力値の計算
        const float raw_output = proportional + integral + derivative;
        // 出力値を制限
        const float clamped_output = min(max(raw_output, -clamping_output), clamping_output);
        // 積分器のanti-windup
        if (raw_output != clamped_output && (raw_output * error > 0)) {
            integral = 0;
        }

        // ログ出力
        if (count % DEBUG_PRINT_INTERVAL == 0) {
            Serial.print("Output: \n");
            Serial.print(
                "output: " + String(clamped_output) + "mA, p: " + String(proportional) + ", i: " + String(integral) +
                ", d: " + String(derivative) + ", current rpm: " + String(rpm) +
                "rpm, target rpm: " + String(target_rpm) + "rpm, error: " + String(error) + "rpm"
            );
            Serial.print("\n\n");
            remote_print("Output: ");
            remote_print(
                "output: " + String(clamped_output) + "mA, p: " + String(proportional) + ", i: " + String(integral) +
                ", d: " + String(derivative) + ", current rpm: " + String(rpm) +
                "rpm, target rpm: " + String(target_rpm) + "rpm, error: " + String(error) + "rpm"
            );
        }
        remote_send_pid_fields(clamped_output, proportional, integral, derivative, target_rpm, error);

        previous_error = error;
        previous_update = millis();

        return clamped_output;
    }
}; // namespace m3508_control::pid_controller