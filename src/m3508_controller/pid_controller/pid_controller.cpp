#include <Arduino.h>
#include "pid_controller.hpp"

namespace m3508_controller::pid_controller
{
    constexpr uint8_t debug_print_cycle = 10;

    /// @brief PIDControllerクラスのコンストラクタ
    /// @param kp pゲイン
    /// @param ki iゲイン
    /// @param kd dゲイン
    /// @param clamping_output 最大出力(積分器のanti-windup用)
    /// @param interval update_output()の実行間隔
    PIDController::PIDController(const float kp, const float ki, const float kd, const float clamping_output, const uint32_t interval)
        : kp(kp), ki(ki), kd(kd), clamping_output(clamping_output), interval(interval)
    {
        count = 0;
    }

    /// @brief フィードバック値を設定
    /// @param angle モータの現在の角度(°)
    /// @param rpm モータの現在の回転数(rpm)
    /// @param amp モータに現在実際に流れている電流量(mA)
    /// @param temp モータの現在の温度(℃)
    void PIDController::set_feedback_values(const float angle, const int16_t rpm, const int16_t amp, const uint8_t temp)
    {
        this->angle = angle;
        this->rpm = rpm;
        this->amp = amp;
        this->temp = temp;
        if (count % debug_print_cycle == 0)
        {
            Serial.print("Received: \n");
            Serial.print("angle: ");
            Serial.print(angle);
            Serial.print(", ");
            Serial.print("rpm: ");
            Serial.print(rpm);
            Serial.print(", ");
            Serial.print("amp: ");
            Serial.print(amp);
            Serial.print(", ");
            Serial.print("temp: ");
            Serial.print(temp);
            Serial.print("\n\n");
        }
    }

    /// @brief 制御目標値を設定
    /// @param target_rpm 制御目標値(rpm)
    void PIDController::set_target_rpm(const int16_t target_rpm)
    {
        this->target_rpm = target_rpm;
    }

    /// @brief 内部状態からPID出力値を計算し、内部状態を更新
    /// @return 出力値
    float PIDController::update_output()
    {
        float current_error = static_cast<float>(target_rpm - rpm);
        integral += current_error * static_cast<float>(interval);
        float derivative = (current_error - previous_error) / static_cast<float>(interval);
        float raw_output = kp * current_error + ki * integral + kd * derivative;

        float clamped_output = min(max(raw_output, -clamping_output), clamping_output);
        if (raw_output != clamped_output && (raw_output * current_error > 0))
        {
            integral = 0;
        }

        if (count % debug_print_cycle == 0)
        {
            Serial.print("Sent: \n");
            Serial.print("output: ");
            Serial.print(clamped_output);
            Serial.print("mA, ");
            Serial.print("p: ");
            Serial.print(kp * current_error);
            Serial.print(", ");
            Serial.print("i: ");
            Serial.print(ki * integral);
            Serial.print(", ");
            Serial.print("d: ");
            Serial.print(kd * derivative);
            Serial.print(", ");
            Serial.print("current rpm: ");
            Serial.print(rpm);
            Serial.print("rpm, ");
            Serial.print("target rpm: ");
            Serial.print(target_rpm);
            Serial.print("rpm, ");
            Serial.print("error: ");
            Serial.print(current_error);
            Serial.print("rpm, ");
            Serial.print("\n\n");
        }

        previous_error = current_error;

        count++;
        return clamped_output;
    }
};