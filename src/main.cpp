#include "bt_communication/bt_communicator.hpp"
#include "bt_communication/bt_interface.hpp"
#include "m3508_controller/m3508_controller.hpp"
#include <Arduino.h>
#include <memory>

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
std::unique_ptr<m3508_controller::M3508Controller> m3508_controller(new m3508_controller::M3508Controller(*bt_interface));

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
    try {
        bt_communicator->loop();
        m3508_controller->loop();
    } catch (const std::exception &e) {
        Serial.print("Unhandled error in loop: ");
        Serial.println(e.what());
        bt_communicator->remote_print("Unhandled error in loop: " + String(e.what()));
    }
    delay(1);
}