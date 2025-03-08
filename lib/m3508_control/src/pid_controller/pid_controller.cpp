#include "pid_controller.hpp"

namespace m3508_control::pid_controller {
    /// @brief pid値のデバッグ出力の間隔(ループ回数)
    constexpr uint8_t DEBUG_PRINT_INTERVAL = 10;
    constexpr uint8_t FEEDBACK_SEND_INTERVAL = 10;

    PIDController::PIDController(
        const float kp,
        const float ki,
        const float kd,
        const float clamping_output,
        const bt_communication::BtPrinter &bt_printer,
        const std::function<void(float output, float proportional, float integral, float derivative, float target_rpm, float error)>
            remote_send_pid_fields
    ) :
        kp(kp),
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
        bt_printer(bt_printer),
        remote_send_pid_fields(remote_send_pid_fields) {}

    void PIDController::set_feedback_values(const float angle, const int16_t rpm, const int16_t amp, const uint8_t temp) {
        static uint32_t count = 0;
        count++;
        this->angle = angle;
        this->rpm = rpm;
        this->amp = amp;
        this->temp = temp;
        if (count % DEBUG_PRINT_INTERVAL == 0) {
            Serial.print("Feedback set: \n");
            Serial.print(
                "angle: " + String(angle) + "deg, rpm: " + String(rpm) + "rpm, amp: " + String(amp) + "mA, temp: " + String(temp) + "deg C"
            );
            Serial.print("\n\n");
            bt_printer.remote_print("Feedback set: ");
            bt_printer.remote_print(
                "angle: " + String(angle) + "deg, rpm: " + String(rpm) + "rpm, amp: " + String(amp) + "mA, temp: " + String(temp) + "deg C"
            );
        }
    }

    void PIDController::set_target_rpm(const int16_t target_rpm) { this->target_rpm = target_rpm; }

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
        const float clamped_output = std::clamp(raw_output, -clamping_output, clamping_output);
        // 積分器のanti-windup
        if (raw_output != clamped_output && (raw_output * error > 0)) {
            integral = 0;
        }

        // ログ出力
        if (count % DEBUG_PRINT_INTERVAL == 0) {
            #ifdef PID_DEBUG
            Serial.print("Output: \n");
            Serial.print(
                "output: "
                + String(clamped_output)
                + "mA, p: "
                + String(proportional)
                + ", i: "
                + String(integral)
                + ", d: "
                + String(derivative)
                + ", current rpm: "
                + String(rpm)
                + "rpm, target rpm: "
                + String(target_rpm)
                + "rpm, error: "
                + String(error)
                + "rpm"
            );
            Serial.print("\n\n");
            bt_printer.remote_print("Output: ");
            bt_printer.remote_print(
                "output: "
                + String(clamped_output)
                + "mA, p: "
                + String(proportional)
                + ", i: "
                + String(integral)
                + ", d: "
                + String(derivative)
                + ", current rpm: "
                + String(rpm)
                + "rpm, target rpm: "
                + String(target_rpm)
                + "rpm, error: "
                + String(error)
                + "rpm"
            );
            #endif
        }

        if (count % FEEDBACK_SEND_INTERVAL == 0) {
            remote_send_pid_fields(clamped_output, proportional, integral, derivative, target_rpm, error);
        }

        previous_error = error;
        previous_update = millis();

        return clamped_output;
    }
}; // namespace m3508_control::pid_controller