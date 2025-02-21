#include "bt_communication/bt_communicator.hpp"
#include "bt_communication/bt_interface.hpp"
#include "m3508_control/m3508_controller.hpp"
#include <Arduino.h>
#include <memory>

constexpr uint32_t CAN_SEND_INTERVAL = 100;
constexpr uint32_t CAN_RECEIVE_INTERVAL = 100;
constexpr uint32_t SERIAL_READ_INTERVAL = 100;

/// Bluetooth通信クラス
std::unique_ptr<bt_communication::BtCommunicator> bt_communicator(new bt_communication::BtCommunicator());
///  Bluetoothでいろいろやり取りする関数をまとめたクラス
std::unique_ptr<bt_communication::BtInterface> bt_interface(new bt_communication::BtInterface(
    [](String text) { bt_communicator->remote_print(text); },
    [](float angle, int16_t rpm, int16_t amp, uint8_t temp) {
        bt_communicator->remote_send_m3508_feedback(angle, rpm, amp, temp);
    },
    [](float output, float p, float i, float d, float target_rpm, float error) {
        bt_communicator->remote_send_m3508_pid_fields(output, p, i, d, target_rpm, error);
    }
));
/// M3508モータの制御クラス
std::unique_ptr<m3508_control::M3508Controller> m3508_controller(new m3508_control::M3508Controller(*bt_interface));

void setup() {
    Serial.begin(115200);
    randomSeed(0); // 乱数生成器のシード値を設定(高速化のため)
    try {
        bt_communicator->setup();
        m3508_controller->setup();
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in setup: ");
        Serial.println(e.what());
        bt_communicator->remote_print("Unhandled error in setup: " + String(e.what()));
    }
}

void loop() {
    static uint32_t count = 0;
    count++;

    try {
        // デバイスが接続されている場合
        if (bt_communicator->is_device_connected()) {
            // モニタのコンソールにサンプル出力
            if (count % 2000 == 0) {
                int random_num = random(255);
                bt_communicator->remote_print("[sample output] random num: ");
                bt_communicator->remote_print(String(random_num));
                Serial.println("[sample output] random num: ");
                Serial.println(String(random_num));
            }
        }

        // M3508に制御量を送信
        if (count % CAN_SEND_INTERVAL == 0) {
            m3508_controller->send_currents();
        }

        // M3508からのフィードバック値を読み取り
        if (count % CAN_RECEIVE_INTERVAL == 0) {
            m3508_controller->read_and_set_feedback();
        }

        // シリアル通信で制御目標値を読み取って設定
        if (count % SERIAL_READ_INTERVAL == 0) {
            m3508_controller->read_serial_and_set_target_rpm();
        }
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in loop: ");
        Serial.println(e.what());
        bt_communicator->remote_print("Unhandled error in loop: " + String(e.what()));
    }

    delay(1);
}