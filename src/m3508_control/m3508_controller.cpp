#include <driver/twai.h>
#include "m3508_controller.hpp"

// モータの制御処理
namespace m3508_control {
    constexpr gpio_num_t CAN_TX = GPIO_NUM_16;
    constexpr gpio_num_t CAN_RX = GPIO_NUM_4;
    constexpr uint16_t CAN_ID = 0x200;

    // PID制御用定数
    constexpr float KP = 0.3;
    constexpr float KI = 0.000012;
    constexpr float KD = 0;
    constexpr float CLAMPING_OUTPUT = 2000;

    M3508Controller::M3508Controller(const bt_communication::BtInterface &bt_interface, const can::CanTransmitter &can_transmitter) :
        pid_controllers({
            {C620Id::C1,
             pid_controller::PIDController(
                 KP,
                 KI,
                 KD,
                 CLAMPING_OUTPUT,
                 bt_interface.remote_print,
                 [&](float output, float proportional, float integral, float derivative, float target_rpm, float error
                 ) {
                     bt_interface.remote_send_pid_fields(
                         C620Id::C1, output, proportional, integral, derivative, target_rpm, error
                     );
                 }
             )},
            {C620Id::C2,
             pid_controller::PIDController(
                 KP,
                 KI,
                 KD,
                 CLAMPING_OUTPUT,
                 bt_interface.remote_print,
                 [&](float output, float proportional, float integral, float derivative, float target_rpm, float error
                 ) {
                     bt_interface.remote_send_pid_fields(
                         C620Id::C2, output, proportional, integral, derivative, target_rpm, error
                     );
                 }
             )},
            {C620Id::C3,
             pid_controller::PIDController(
                 KP,
                 KI,
                 KD,
                 CLAMPING_OUTPUT,
                 bt_interface.remote_print,
                 [&](float output, float proportional, float integral, float derivative, float target_rpm, float error
                 ) {
                     bt_interface.remote_send_pid_fields(
                         C620Id::C3, output, proportional, integral, derivative, target_rpm, error
                     );
                 }
             )},
            {C620Id::C4,
             pid_controller::PIDController(
                 KP,
                 KI,
                 KD,
                 CLAMPING_OUTPUT,
                 bt_interface.remote_print,
                 [&](float output, float proportional, float integral, float derivative, float target_rpm, float error
                 ) {
                     bt_interface.remote_send_pid_fields(
                         C620Id::C4, output, proportional, integral, derivative, target_rpm, error
                     );
                 }
             )},
        }),
        target_velocity(utils::Vec2(0, 0)),
        command_currents{0, 0, 0, 0},
        bt_interface(bt_interface),
        can_transmitter(can_transmitter) {}

    /// @brief 使う前に呼び出す！(CANの初期化など)
    void M3508Controller::setup() {
    }

    /// @brief PIDで電流値を計算してM3508にCANで送信
    void M3508Controller::send_currents() {
        command_currents[0] = pid_controllers.at(C620Id::C1).update_output();
        command_currents[1] = pid_controllers.at(C620Id::C2).update_output();
        command_currents[2] = pid_controllers.at(C620Id::C3).update_output();
        command_currents[3] = pid_controllers.at(C620Id::C4).update_output();

        std::array<uint8_t, 8> tx_buf;
        milli_amperes_to_bytes(command_currents, tx_buf.data());

        can_transmitter.transmit(CAN_ID, tx_buf);
    }

    /// @brief M3508からのフィードバックを読み取って、PID制御器に設定
    void M3508Controller::set_feedback(const can::CanId rx_id, const std::array<uint8_t, 8> rx_buf) {
        C620Id rx_c620_id = static_cast<C620Id>(rx_id - 0x200);

        float angle;
        int16_t rpm;
        int16_t amp;
        uint8_t temp;
        derive_feedback_fields(rx_buf.data(), &angle, &rpm, &amp, &temp);
        pid_controllers.at(rx_c620_id).set_feedback_values(angle, rpm, amp, temp);

        bt_interface.remote_send_feedback(rx_c620_id, angle, rpm, amp, temp);
    }

    /// @brief シリアル通信を読み取ってPIDの目標値を設定
    void M3508Controller::read_serial_and_set_target_rpm() {
        if (Serial.available()) {
            delay(1); // 一連のシリアル信号をすべて受信するまで待つ
            String input_string = "";
            while (Serial.available() > 0) {
                char input_char = Serial.read();
                input_string.concat(input_char);
            }
            pid_controllers.at(C620Id::C1).set_target_rpm(input_string.toInt());
            pid_controllers.at(C620Id::C2).set_target_rpm(input_string.toInt());
            pid_controllers.at(C620Id::C3).set_target_rpm(input_string.toInt());
            pid_controllers.at(C620Id::C4).set_target_rpm(input_string.toInt());
            Serial.print("Set target rpm to: " + input_string);
            Serial.print("\n\n");

            bt_interface.remote_print("Set target rpm to: " + input_string);
        }
    }

    void M3508Controller::set_target_velocity(const utils::Vec2 &target_velocity) {
        this->target_velocity = target_velocity;

        float target_rpm_1, target_rpm_2, target_rpm_3, target_rpm_4;
        calc_target_rpms(target_velocity, &target_rpm_1, &target_rpm_2, &target_rpm_3, &target_rpm_4);
        pid_controllers.at(C620Id::C1).set_target_rpm(target_rpm_1);
        pid_controllers.at(C620Id::C2).set_target_rpm(target_rpm_2);
        pid_controllers.at(C620Id::C3).set_target_rpm(target_rpm_3);
        pid_controllers.at(C620Id::C4).set_target_rpm(target_rpm_4);
    }

    /// @brief 4つの電流値を、CANで速度コントローラに送信するデータへ変換
    /// @param milli_amperes 4つの-20000\~20000の電流値(mA)を格納した配列
    /// (要素番号と速度コントローラIDが対応)
    /// @param out_tx_buf 結果の書き込み用配列
    void M3508Controller::milli_amperes_to_bytes(const int32_t milli_amperes[4], uint8_t out_tx_buf[8]) {
        uint8_t i;
        for (i = 0; i < 4; i++) {
            int32_t milli_ampere = milli_amperes[i] * 16384 / 20000;
            uint8_t upper = (milli_ampere >> 8) & 0xFF;
            uint8_t lower = milli_ampere & 0xFF;
            out_tx_buf[i * 2] = upper;
            out_tx_buf[i * 2 + 1] = lower;
        }
    }

    /// @brief 速度コントローラから受け取ったデータから、フィードバック値を導出
    /// @param rx_buf CANで受信した配列
    /// @param out_angle ロータの角度(0°\~360°) (結果書き込み用)
    /// @param out_rpm 回転速度(rpm) (結果書き込み)
    /// @param out_amp 実際のトルク電流(?) (結果書き込み用)
    /// @param out_temp モータの温度(℃) (結果書き込み用)
    ///
    /// TODO: std::arrayを受け取るようにする
    void M3508Controller::derive_feedback_fields(
        const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp
    ) {
        *out_angle = (float)(rx_buf[0] << 8 | rx_buf[1]) * 360.0f / 8191.0f;
        *out_rpm = rx_buf[2] << 8 | rx_buf[3];
        *out_amp = rx_buf[4] << 8 | rx_buf[5];
        *out_temp = rx_buf[6];
    }

    void M3508Controller::calc_target_rpms(
        const utils::Vec2 &target_velocity,
        float *out_target_rpm_1,
        float *out_target_rpm_2,
        float *out_target_rpm_3,
        float *out_target_rpm_4
    ) {
        constexpr float one_over_root_2 = 0.70710678118f; // 1.0 / sqrt(2.0)
        constexpr float wheel_radius = 0.051f;            // 車輪の半径(m)
        constexpr float reduction_ratio = 19.2032085561;  // 減速比

        // TODO: 機体の回転を考慮
        *out_target_rpm_1 = one_over_root_2 * (-target_velocity.x + target_velocity.y) / wheel_radius * 60.0f /
                            (2.0f * M_PI) * reduction_ratio;
        *out_target_rpm_2 = one_over_root_2 * (-target_velocity.x - target_velocity.y) / wheel_radius * 60.0f /
                            (2.0f * M_PI) * reduction_ratio;
        *out_target_rpm_3 = one_over_root_2 * (target_velocity.x - target_velocity.y) / wheel_radius * 60.0f /
                            (2.0f * M_PI) * reduction_ratio;
        *out_target_rpm_4 = one_over_root_2 * (target_velocity.x + target_velocity.y) / wheel_radius * 60.0f /
                            (2.0f * M_PI) * reduction_ratio;
    }
} // namespace m3508_control