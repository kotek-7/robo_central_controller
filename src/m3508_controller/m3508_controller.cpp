#include "m3508_controller.hpp"
#include "bt_communication/bt_interface.hpp"
#include "pid_controller/pid_controller.hpp"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

// モータの制御処理
namespace m3508_controller {
    constexpr uint8_t CAN_TX = 16;
    constexpr uint8_t CAN_RX = 4;

    constexpr uint16_t CAN_ID = 0x200;
    constexpr uint32_t CAN_SEND_INTERVAL = 100;
    constexpr uint32_t CAN_RECEIVE_INTERVAL = 100;
    constexpr uint32_t SERIAL_READ_INTERVAL = 100;

    // PID制御用定数
    constexpr float KP = 0.5f;
    constexpr float KI = 0.0003f;
    constexpr float KD = 40;
    constexpr float CLAMPING_OUTPUT = 2000;

    /// @param remote_print モニタにテキストを送信する関数
    /// @param remote_send_feedback モニタにフィードバック値を送信する関数
    /// @param remote_send_pid_fields モニタにPID制御値を送信する関数
    M3508Controller::M3508Controller(const bt_communication::BtInterface &bt_interface)
        : pid_controller(KP, KI, KD, CLAMPING_OUTPUT, CAN_SEND_INTERVAL, bt_interface),
          bt_interface(bt_interface),
          previous_can_send_millis(0),
          previous_can_receive_millis(0),
          previous_serial_read_millis(0) {}

    /// @brief セットアップ
    void M3508Controller::setup() {
        ESP32Can.setRxQueueSize(5);
        ESP32Can.setTxQueueSize(5);

        ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));

        ESP32Can.setPins(CAN_TX, CAN_RX);
        if (ESP32Can.begin()) {
            Serial.println("Init OK!");
            bt_interface.remote_print("Init OK!");
        } else {
            Serial.println("Init Fail!");
            bt_interface.remote_print("Init Fail!");
        }
        previous_can_send_millis = millis();
        previous_can_receive_millis = millis();
    }

    /// @brief メインループ
    void M3508Controller::loop() {
        /// @brief モータに送信する電流値(mA)
        static int32_t command_currents[4] = {0, 0, 0, 0};

        // CANで制御量を送信
        if ((millis() - previous_can_send_millis) > CAN_SEND_INTERVAL) {
            command_currents[0] = pid_controller.update_output();
            uint8_t tx_buf[8];
            milli_amperes_to_bytes(command_currents, tx_buf);

            Serial.println("Sending: ");
            for (byte i = 0; i < 8; i++) {
                char msg_string[128];
                sprintf(msg_string, " 0x%.2X", tx_buf[i]);
                Serial.print(msg_string);
            }
            Serial.println();

            CanFrame tx_frame;
            tx_frame.identifier = CAN_ID;
            tx_frame.extd = 0;
            tx_frame.data_length_code = 8;
            tx_frame.data[0] = tx_buf[0];
            tx_frame.data[1] = tx_buf[1];
            tx_frame.data[2] = tx_buf[2];
            tx_frame.data[3] = tx_buf[3];
            tx_frame.data[4] = tx_buf[4];
            tx_frame.data[5] = tx_buf[5];
            tx_frame.data[6] = tx_buf[6];
            tx_frame.data[7] = tx_buf[7];
            bool writeFrameOk = ESP32Can.writeFrame(tx_frame);

            Serial.println("Sent! result:" + String(writeFrameOk));
            for (uint8_t i = 0; i < 8; i++) {
                Serial.print(tx_frame.data[i]);
                Serial.print(" ");
            }
            Serial.println();

            previous_can_send_millis = millis();
        }

        // CANでフィードバック値を受信
        if ((millis() - previous_can_receive_millis) > CAN_RECEIVE_INTERVAL) {
            Serial.println("Checking for can received buf!");
            CanFrame rx_frame;
            if (ESP32Can.readFrame(rx_frame, 0)) {
                Serial.println("Can frame received!");
                uint32_t rx_id = rx_frame.identifier;

                uint32_t controller_id = rx_id - 0x200;
                float angle;
                int16_t rpm;
                int16_t amp;
                uint8_t temp;
                derive_feedback_fields(rx_frame.data, &angle, &rpm, &amp, &temp);

                Serial.println("Angle: " + String(angle));
                Serial.println("Rpm: " + String(rpm));
                Serial.println("Amp: " + String(amp));
                Serial.println("Temp: " + String(temp));
                bt_interface.remote_print("Received!");

                bt_interface.remote_send_feedback(angle, rpm, amp, temp);
                pid_controller.set_feedback_values(angle, rpm, amp, temp);
            } else {
                Serial.println("No frame received!");
            }

            previous_can_receive_millis = millis();
        }

        // シリアル通信で制御目標値を受信
        if ((millis() - previous_serial_read_millis) > SERIAL_READ_INTERVAL) {
            if (Serial.available()) {
                delay(1); // 一連のシリアル信号をすべて受信するまで待つ
                String input_string = "";
                while (Serial.available() > 0) {
                    char input_char = Serial.read();
                    input_string.concat(input_char);
                }
                pid_controller.set_target_rpm(input_string.toInt());
                Serial.print("Set target rpm to: ");
                Serial.print(input_string);
                Serial.print("\n\n");
                bt_interface.remote_print("Set target rpm to: " + input_string);
            }
            previous_serial_read_millis = millis();
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
} // namespace m3508_controller