#include "m3508_controller.hpp"
#include "bt_communication/bt_interface.hpp"
#include "pid_controller/pid_controller.hpp"
#include <Arduino.h>
#include <driver/twai.h>

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

    M3508Controller::M3508Controller(const bt_communication::BtInterface &bt_interface)
        : pid_controller(KP, KI, KD, CLAMPING_OUTPUT, bt_interface),
          bt_interface(bt_interface),
          command_currents{0, 0, 0, 0} {}

    /// @brief 使う前に呼び出す！(CANの初期化など)
    void M3508Controller::setup() {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
            Serial.println("Driver install OK!");
            bt_interface.remote_print("Driver install OK!");
        } else {
            Serial.println("Driver install fail!");
            bt_interface.remote_print("Driver install fail!");
            return;
        }

        if (twai_start() == ESP_OK) {
            Serial.println("Driver start OK!");
            bt_interface.remote_print("Driver start OK!");
        } else {
            Serial.println("Driver start fail!");
            bt_interface.remote_print("Driver start fail!");
            return;
        }
    }

    /// @brief M3508にCANで電流値を送信
    void M3508Controller::send_currents() {
        command_currents[0] = pid_controller.update_output();
        uint8_t tx_buf[8];
        milli_amperes_to_bytes(command_currents, tx_buf);

        twai_message_t tx_message;
        tx_message.identifier = CAN_ID;
        tx_message.extd = 0;
        tx_message.rtr = 0;
        tx_message.ss = 0;
        tx_message.self = 0;
        tx_message.dlc_non_comp = 0;
        tx_message.data_length_code = 8;
        for (uint8_t i = 0; i < 8; i++) {
            tx_message.data[i] = tx_buf[i];
        }

        const auto tx_result = twai_transmit(&tx_message, 0);
        if (tx_result != ESP_OK) {
            Serial.println("Transmit Fail: The TX queue is full!");
            bt_interface.remote_print("Transmit Fail: The TX queue is full!");
        }
    }

    /// @brief M3508からのフィードバックを読み取って、PID制御器に設定
    void M3508Controller::read_and_set_feedback() {
        twai_message_t rx_message;
        const auto rx_result = twai_receive(&rx_message, 0);
        if (rx_result != ESP_OK) {
            Serial.println("Receive Fail: The RX queue is empty!");
            bt_interface.remote_print("Receive Fail: The RX queue is empty!");
        }
        if (rx_message.rtr) {
            Serial.println("Receive Fail: The received message is a remote frame!");
            bt_interface.remote_print("Receive Fail: The received message is a remote frame!");
            return;
        }
        if (rx_message.extd) {
            Serial.println("Receive Fail: The received message is an extended frame!");
            bt_interface.remote_print("Receive Fail: The received message is an extended frame!");
            return;
        }

        uint32_t rx_id = rx_message.identifier;

        uint32_t controller_id = rx_id - 0x200;
        float angle;
        int16_t rpm;
        int16_t amp;
        uint8_t temp;
        derive_feedback_fields(rx_message.data, &angle, &rpm, &amp, &temp);
        pid_controller.set_feedback_values(angle, rpm, amp, temp);

        bt_interface.remote_send_feedback(angle, rpm, amp, temp);
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
            pid_controller.set_target_rpm(input_string.toInt());
            Serial.print("Set target rpm to: " + input_string);
            Serial.print("\n\n");

            bt_interface.remote_print("Set target rpm to: " + input_string);
        }
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
    void M3508Controller::derive_feedback_fields(
        const uint8_t rx_buf[8], float *out_angle, int16_t *out_rpm, int16_t *out_amp, uint8_t *out_temp
    ) {
        *out_angle = (float)(rx_buf[0] << 8 | rx_buf[1]) * 360.0f / 8191.0f;
        *out_rpm = rx_buf[2] << 8 | rx_buf[3];
        *out_amp = rx_buf[4] << 8 | rx_buf[5];
        *out_temp = rx_buf[6];
    }
} // namespace m3508_control